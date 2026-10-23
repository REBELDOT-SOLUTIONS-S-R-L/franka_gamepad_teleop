#include "franka_gamepad_teleop/core.hpp"
#include "franka_gamepad_teleop/input.hpp"
#include "franka_gamepad_teleop/gripper.hpp"
#include "franka_gamepad_teleop/logging.hpp"
#include "franka_gamepad_teleop/force_control.hpp"
#include <yaml-cpp/yaml.h>
#include <franka/robot.h>
#include <franka/control_types.h>
#include <franka/model.h>
#include <chrono>
#include <algorithm>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>

namespace {
volatile std::sig_atomic_t interrupted=0;
void signal_handler(int) { interrupted=1; }
struct Options {
  std::string config,ip,log,mode="preview";
  double duration=0;
};
Options options(int argc,char** argv) {
  Options o; bool mode_set=false;
  for(int i=1;i<argc;++i) {
    const std::string a=argv[i];
    if(a=="--help"||a=="-h") {
      std::cout<<"Usage: gamepad_teleop --config FILE [--robot-ip IP]\n"
        "  --inspect     Read gamepad only; no robot connection\n"
        "  --simulate    Integrate a synthetic starting TCP; no robot connection\n"
        "  --read-state  Print the robot's current configured TCP, then exit\n"
        "  --preview     Read robot state and show target deltas; no motion (default)\n"
        "  --execute     Run real Delta/Speed/Force control; starts disarmed\n"
        "  --validate-config  Validate YAML only, without connecting to devices\n"
        "  --log FILE   Record diagnostic CSV at up to 250 Hz; no control changes\n"
        "  --duration S  Exit after S seconds (0 means unlimited)\n";
      std::exit(0);
    } else if(a=="--config"||a=="--robot-ip"||a=="--duration"||a=="--log") {
      if(++i>=argc) throw std::runtime_error("Missing value after "+a);
      if(a=="--config") o.config=argv[i];
      else if(a=="--robot-ip") o.ip=argv[i];
      else if(a=="--log") o.log=argv[i];
      else { size_t used=0; o.duration=std::stod(argv[i],&used); if(used!=std::string(argv[i]).size()||!std::isfinite(o.duration)||o.duration<0) throw std::runtime_error("Invalid duration"); }
    } else if(a=="--inspect"||a=="--simulate"||a=="--read-state"||a=="--preview"||a=="--execute"||a=="--validate-config") {
      if(mode_set) throw std::runtime_error("Choose one operating mode");
      o.mode=a.substr(2); mode_set=true;
    } else throw std::runtime_error("Unknown argument: "+a);
  }
  if(o.config.empty()) throw std::runtime_error("--config is required");
  if(!o.log.empty()&&o.mode!="execute"&&o.mode!="preview"&&o.mode!="simulate")
    throw std::runtime_error("--log requires execute, preview or simulate mode");
  if(o.mode!="inspect"&&o.mode!="simulate"&&o.mode!="validate-config"&&o.ip.empty()) throw std::runtime_error("--robot-ip is required");
  return o;
}
void pose(const std::string& label,const gamepad::Pose& p) {
  Eigen::Map<const Eigen::Matrix4d> t(p.data());
  const Eigen::Quaterniond q(t.topLeftCorner<3,3>());
  std::cout<<label<<" base XYZ [m]: "<<p[12]<<", "<<p[13]<<", "<<p[14]
    <<"; quaternion [x y z w]: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
}
struct Telemetry {
  std::atomic<bool> done{false},armed{false},stopping{false},refused{false};
  std::atomic<int> grip{0};
  std::atomic<int> mode{0};
  std::atomic<bool> mode_refused{false},mode_ready{false};
  std::atomic<uint32_t> arm_reason{0};
  std::array<std::atomic<double>,16> target{};
  void store(const gamepad::Step& s) {
    for(size_t i=0;i<16;++i) target[i].store(s.target[i]);
    armed.store(s.armed); stopping.store(s.shutdown);
    if(s.arm_refused) { arm_reason.store(s.arm_refusal_reason); refused.store(true); }
    if(s.grip!=gamepad::Grip::None) grip.store(static_cast<int>(s.grip));
    mode.store(static_cast<int>(s.mode)); mode_ready.store(s.mode_ready);
    if(s.mode_refused) mode_refused.store(true);
  }
};
class Reporter {
 public:
  Reporter(Telemetry& t,gamepad::GripperChannel& g):data_(t),gripper_(g),thread_([this] {
    bool was_armed=false,was_stopping=false,was_ready=false; unsigned ticks=0; int previous_mode=-1;
    while(!data_.done.load()) {
      const bool armed=data_.armed.load(),stopping=data_.stopping.load();
      if(armed!=was_armed) { std::cout<<(armed?"ARMED":"DISARMED: decelerating to hold")<<std::endl; was_armed=armed; }
      if(stopping&&!was_stopping) { std::cout<<"STOPPING: input/robot timing, gripper fault, duration or exit request"<<std::endl; was_stopping=true; }
      if(data_.refused.exchange(false)) {
        const auto reason=data_.arm_reason.load();
        std::cout<<"Arm refused:";
        if(reason&1) std::cout<<" motion controls not neutral;";
        if(reason&2) std::cout<<" app velocity/wrench still decelerating;";
        if(reason&4) std::cout<<" gripper busy;";
        if(reason&8) std::cout<<" release X and Y after mode change;";
        if(reason&16) std::cout<<" controller transition pending;";
        if(reason&32) std::cout<<" waiting for measured robot rest;";
        if(reason&64) std::cout<<" release A/B/X;";
        std::cout<<" press Y again after readiness"<<std::endl;
      }
      const int mode=data_.mode.load(); const bool ready=data_.mode_ready.load();
      if(mode!=previous_mode||ready!=was_ready) {
        std::cout<<"Control mode: "<<gamepad::mode_name(static_cast<gamepad::ControlMode>(mode))
          <<(ready?" (ready)":" (transitioning; disarmed)")<<std::endl;
        previous_mode=mode; was_ready=ready;
      }
      if(data_.mode_refused.exchange(false)) std::cout<<"Mode change refused: disarm, wait for robot rest, release motion controls and Y/A/B, and wait for gripper/controller readiness"<<std::endl;
      const int grip=data_.grip.exchange(0);
      if(grip) std::cout<<(grip==1?"GRIPPER intent: contact grasp":"GRIPPER intent: open fully")<<std::endl;
      if(++ticks%10==0) {
        gamepad::Pose p; for(size_t i=0;i<16;++i) p[i]=data_.target[i].load();
        pose("Target",p);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    gripper_.authorized_epoch.store(0);
  }) {}
  ~Reporter() { data_.done.store(true); if(thread_.joinable()) thread_.join(); }
 private:
  Telemetry& data_; gamepad::GripperChannel& gripper_; std::thread thread_;
};
}
int main(int argc,char** argv) {
  using namespace gamepad;
  try {
    const auto opt=options(argc,argv);
    const auto config=load_config(opt.config);
    if(opt.mode=="validate-config") { std::cout<<"Configuration valid: initial mode "<<mode_name(config.initial_mode)<<std::endl; return 0; }
    std::signal(SIGINT,signal_handler); std::signal(SIGTERM,signal_handler);
    std::cout<<std::fixed<<std::setprecision(6);
    const auto began=steady_time();
    auto ending=[&] { return interrupted||(opt.duration>0&&steady_time()-began>=opt.duration); };
    if(opt.mode=="inspect") {
      Device device(config); std::cout<<device.description()<<std::endl;
      do {
        const auto s=device.sample();
        if(!s.connected) throw std::runtime_error("Gamepad disconnected");
        std::cout<<"axes [left_x left_y right_x right_y L2 R2]: ";
        for(double x:s.axes) std::cout<<x<<" ";
        std::cout<<" buttons_mask="<<s.buttons<<" mapped [base_X base_Y base_Z tcp_Rx tcp_Ry tcp_Rz tcp_Z]: ";
        for(double x:map_input(config,s)) std::cout<<x<<" ";
        std::cout<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } while(!ending());
      return 0;
    }
    std::unique_ptr<franka::Robot> robot;
    std::optional<franka::RobotState> initial_state;
    Pose initial=identity_pose(); initial[12]=.5; initial[14]=.4;
    if(opt.mode!="simulate") {
      robot=std::make_unique<franka::Robot>(opt.ip);
      const auto state=robot->readOnce(); initial=state.O_T_EE;
      initial_state=state;
      if(!valid_pose(initial)) throw std::runtime_error("Robot supplied an invalid TCP transform");
      pose("Current configured TCP (O_T_EE)",initial);
      if(opt.mode=="read-state") return 0;
    } else pose("Synthetic initial TCP",initial);
    Reader reader(config); std::cout<<reader.description<<std::endl;
    std::unique_ptr<DiagnosticLog> diagnostics;
    if(!opt.log.empty()) {
      YAML::Node metadata;
      metadata["schema_version"]=2; metadata["mode"]=opt.mode; metadata["robot_ip"]=opt.ip;
      metadata["control_modes"]="0=Delta, 1=Speed, 2=Force; control_backend is the API active for this sample";
      metadata["arm_refusal_bits"]="1=nonneutral controls,2=app braking,4=gripper busy,8=release after mode change,16=backend pending,32=measured rest pending,64=A/B/X held";
      metadata["command_units"]="requested_velocity: base-frame m/s, rad/s; requested_wrench/task_wrench: base-frame N,Nm at TCP; sent_torque: Nm without gravity/friction";
      metadata["compiled"]=std::string(__DATE__)+" "+__TIME__;
      metadata["sample_rate_hz_max"]=250;
      metadata["pose_layout"]="column-major 4x4; translation indices 12,13,14; metres";
      metadata["mapped_order"]="base_X base_Y base_Z tcp_Rx tcp_Ry tcp_Rz tcp_Z (normalized)";
      metadata["alignment"]="Input and app target belong to this callback; commanded/desired/measured are from its incoming RobotState, before this target is processed by the SDK/robot.";
      metadata["config"]=YAML::LoadFile(opt.config); metadata["controller_mapping"]=reader.description;
      if(initial_state) {
        auto array=[&](const char* name,const auto& values) { for(double v:values) metadata["initial_robot"][name].push_back(v); };
        array("O_T_EE",initial_state->O_T_EE); array("F_T_EE",initial_state->F_T_EE);
        array("F_T_NE",initial_state->F_T_NE); array("NE_T_EE",initial_state->NE_T_EE);
        array("EE_T_K",initial_state->EE_T_K); array("F_x_Cee",initial_state->F_x_Cee);
        array("F_x_Cload",initial_state->F_x_Cload);
        metadata["initial_robot"]["m_ee"]=initial_state->m_ee;
        metadata["initial_robot"]["m_load"]=initial_state->m_load;
      }
      YAML::Emitter emitter; emitter<<metadata;
      diagnostics=std::make_unique<DiagnosticLog>(opt.log,emitter.c_str());
      std::cout<<"Diagnostic CSV: "<<opt.log<<std::endl;
    }
    Teleop teleop(config,initial);
    GripperChannel channel;
    std::unique_ptr<GripperWorker> gripper;
    if(opt.mode=="execute"&&config.gripper) gripper=std::make_unique<GripperWorker>(opt.ip,config,channel);
    Telemetry telemetry; telemetry.mode.store(static_cast<int>(config.initial_mode));
    for(size_t i=0;i<16;++i) telemetry.target[i].store(initial[i]);
    Reporter reporter(telemetry,channel);
    uint32_t sequence=0;
    Input last_input=reader.input.load();
    auto tick=[&](double dt,const Feedback& feedback) {
      const auto input=reader.input.load();
      // A brief seqlock collision keeps the preceding snapshot and its original
      // timestamp. A stalled producer still expires at the configured deadline.
      if(input.stamp>0) last_input=input;
      const auto s=teleop.update(last_input,steady_time(),dt,channel.pause.load()||channel.pending(),ending()||channel.fault.load(),feedback);
      channel.authorized_epoch.store(s.armed?s.epoch:0);
      channel.at_rest.store(s.stopped&&feedback.still&&s.mode_ready);
      if(s.grip!=Grip::None) {
        if(gripper) channel.publish(s.epoch,++sequence,s.grip);
        // Preview/simulation expose intent without opening a Gripper connection.
      }
      telemetry.store(s); return s;
    };
    uint64_t diagnostic_cycle=0;
    ControlMode active_backend=config.initial_mode;
    double last_diagnostic_time=-1;
    auto record=[&](const Step& step,double period,const franka::RobotState* state=nullptr,const ForceOutput* force=nullptr) {
      if(!diagnostics) return;
      ++diagnostic_cycle;
      const double now=steady_time();
      const double clock=state?state->time.toSec():now;
      if(last_diagnostic_time>=0&&clock-last_diagnostic_time<.003999) return;
      last_diagnostic_time=clock;
      DiagnosticSample sample;
      sample.cycle=diagnostic_cycle; sample.monotonic=now; sample.robot_time=state?clock:0;
      sample.period=period; sample.input=last_input; sample.mapped=map_input(config,last_input); sample.step=step;
      sample.backend=active_backend;
      if(force) { sample.sent_torque=force->torque; sample.task_wrench=force->task_wrench; }
      if(state) {
        sample.has_robot=true; sample.robot_mode=static_cast<int>(state->robot_mode);
        sample.measured=state->O_T_EE; sample.commanded=state->O_T_EE_c; sample.desired=state->O_T_EE_d;
        sample.q=state->q; sample.q_d=state->q_d; sample.dq=state->dq;
        sample.tau_external=state->tau_ext_hat_filtered;
        sample.commanded_twist=state->O_dP_EE_c; sample.commanded_acceleration=state->O_ddP_EE_c;
        sample.external_wrench=state->O_F_ext_hat_K; sample.command_success_rate=state->control_command_success_rate;
      }
      diagnostics->record(sample);
    };
    std::cout<<"Run: "<<opt.mode<<"; control "<<mode_name(config.initial_mode)<<"; "<<config.speed*1000<<" mm/s, "<<config.rotation*180/kPi
      <<" deg/s; Force "<<config.force_control.max_force<<" N, "<<config.force_control.max_torque
      <<" Nm; DISARMED; Y toggles arm; X cycles mode while still/disarmed; A grasp, B open"<<std::endl;
    if(opt.mode=="execute") {
      auto model=robot->loadModel();
      ForceController force_controller(config.force_control);
      bool finished=false,from_force=false,force_fault=false;
      auto measured_velocity=[&](const franka::RobotState& state,const std::array<double,42>& jacobian) {
        std::array<double,6> velocity{};
        Eigen::Map<Eigen::Matrix<double,6,1>>(velocity.data())=
          Eigen::Map<const Eigen::Matrix<double,6,7>>(jacobian.data())*
          Eigen::Map<const Eigen::Matrix<double,7,1>>(state.dq.data());
        return velocity;
      };
      try {
        while(!finished) {
          active_backend=teleop.mode();
          bool first=true;
          RestMonitor rest;
          auto feedback=[&](const franka::RobotState& state,double dt,const std::array<double,42>& jacobian) {
            Feedback f; f.pose=state.O_T_EE; f.valid=true;
            f.backend_ready=teleop.mode()==active_backend;
            f.measured_velocity=measured_velocity(state,jacobian);
            f.still=rest.update(dt,state.dq,f.measured_velocity,state.O_dP_EE_c,active_backend!=ControlMode::Force);
            f.rest_elapsed=rest.elapsed();f.rest_velocity=rest.filtered_velocity();
            return f;
          };
          auto done=[&](const Step& s,const Feedback& f) {
            const bool shutdown=s.shutdown&&s.stopped&&f.still;
            if(shutdown) finished=true;
            return shutdown||(teleop.mode()!=active_backend&&s.stopped&&f.still);
          };
          if(active_backend==ControlMode::Delta) {
            robot->control([&](const franka::RobotState& state,franka::Duration period)->franka::CartesianPose {
              if(first) { teleop.seed_command_pose(from_force?state.O_T_EE:state.O_T_EE_c); first=false; }
              const auto j=model.zeroJacobian(franka::Frame::kEndEffector,state);
              const auto f=feedback(state,period.toSec(),j); const auto s=tick(period.toSec(),f);
              record(s,period.toSec(),&state);
              const franka::CartesianPose command(s.target);
              return done(s,f)?franka::MotionFinished(command):command;
            },franka::ControllerMode::kCartesianImpedance,true,config.cutoff);
          } else if(active_backend==ControlMode::Speed) {
            robot->control([&](const franka::RobotState& state,franka::Duration period)->franka::CartesianVelocities {
              if(first) { teleop.seed_command_pose(state.O_T_EE); first=false; }
              const auto j=model.zeroJacobian(franka::Frame::kEndEffector,state);
              const auto f=feedback(state,period.toSec(),j); const auto s=tick(period.toSec(),f);
              record(s,period.toSec(),&state);
              const franka::CartesianVelocities command(s.velocity);
              return done(s,f)?franka::MotionFinished(command):command;
            },franka::ControllerMode::kCartesianImpedance,true,config.cutoff);
          } else {
            robot->control([&](const franka::RobotState& state,franka::Duration period)->franka::Torques {
              ForceState fs; fs.pose=state.O_T_EE; fs.q=state.q; fs.dq=state.dq;
              fs.jacobian=model.zeroJacobian(franka::Frame::kEndEffector,state);
              fs.coriolis=model.coriolis(state); fs.previous_torque=state.tau_J_d;
              if(first) { teleop.seed_command_pose(state.O_T_EE); force_controller.seed(fs); first=false; }
              const auto f=feedback(state,period.toSec(),fs.jacobian); const auto s=tick(period.toSec(),f);
              const auto force=force_controller.update(s,fs,period.toSec());
              record(s,period.toSec(),&state,&force);
              if(!force.valid) { force_fault=true; finished=true; return franka::MotionFinished(franka::Torques(state.tau_J_d)); }
              const franka::Torques command(force.torque);
              return done(s,f)?franka::MotionFinished(command):command;
            },true,config.cutoff);
          }
          from_force=active_backend==ControlMode::Force;
        }
        if(force_fault) throw std::runtime_error("Force control received invalid pose/model/torque data; session ended");
      } catch(...) { channel.authorized_epoch.store(0); throw; }
    } else if(opt.mode=="preview") {
      double previous=0;
      RestMonitor rest;
      robot->read([&](const franka::RobotState& state) {
        const double now=state.time.toSec(); const double dt=previous==0?0:now-previous; previous=now;
        Feedback f; f.pose=state.O_T_EE; f.valid=true;
        // Read-only Robot::read is not the 1 kHz control callback. Joint
        // velocities gate stillness; clamp preview integration steps only.
        std::array<double,6> zero{};
        f.still=rest.update(dt,state.dq,zero,zero,false);
        f.rest_elapsed=rest.elapsed();f.rest_velocity=rest.filtered_velocity();
        const auto s=tick(std::clamp(dt,0.,.020),f); active_backend=s.mode;
        record(s,dt,&state); return !(s.shutdown&&s.stopped);
      });
    } else {
      auto previous=steady_time();
      Step last; last.target=initial;
      while(true) {
        const auto now=steady_time(); const double dt=now-previous;
        Feedback f; f.pose=last.target; f.valid=true; f.still=last.stopped;
        const auto s=tick(dt,f); previous=now; last=s; active_backend=s.mode;
        record(s,dt);
        if(s.shutdown&&s.stopped) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    channel.authorized_epoch.store(0);
    std::cout<<"Session ended at rest"<<std::endl;
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"gamepad_teleop: "<<error.what()<<std::endl; return 1;
  }
}
