#include "franka_gamepad_teleop/core.hpp"
#include "franka_gamepad_teleop/input.hpp"
#include "franka_gamepad_teleop/gripper.hpp"
#include <franka/robot.h>
#include <franka/control_types.h>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

namespace {
volatile std::sig_atomic_t interrupted=0;
void signal_handler(int) { interrupted=1; }
struct Options {
  std::string config,ip,mode="preview";
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
        "  --execute     Run real Cartesian pose control; starts disarmed\n"
        "  --validate-config  Validate YAML only, without connecting to devices\n"
        "  --duration S  Exit after S seconds (0 means unlimited)\n";
      std::exit(0);
    } else if(a=="--config"||a=="--robot-ip"||a=="--duration") {
      if(++i>=argc) throw std::runtime_error("Missing value after "+a);
      if(a=="--config") o.config=argv[i];
      else if(a=="--robot-ip") o.ip=argv[i];
      else { size_t used=0; o.duration=std::stod(argv[i],&used); if(used!=std::string(argv[i]).size()||!std::isfinite(o.duration)||o.duration<0) throw std::runtime_error("Invalid duration"); }
    } else if(a=="--inspect"||a=="--simulate"||a=="--read-state"||a=="--preview"||a=="--execute"||a=="--validate-config") {
      if(mode_set) throw std::runtime_error("Choose one operating mode");
      o.mode=a.substr(2); mode_set=true;
    } else throw std::runtime_error("Unknown argument: "+a);
  }
  if(o.config.empty()) throw std::runtime_error("--config is required");
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
  std::array<std::atomic<double>,16> target{};
  void store(const gamepad::Step& s) {
    for(size_t i=0;i<16;++i) target[i].store(s.target[i]);
    armed.store(s.armed); stopping.store(s.shutdown);
    if(s.arm_refused) refused.store(true);
    if(s.grip!=gamepad::Grip::None) grip.store(static_cast<int>(s.grip));
  }
};
class Reporter {
 public:
  Reporter(Telemetry& t,gamepad::GripperChannel& g):data_(t),gripper_(g),thread_([this] {
    bool was_armed=false,was_stopping=false; unsigned ticks=0;
    while(!data_.done.load()) {
      const bool armed=data_.armed.load(),stopping=data_.stopping.load();
      if(armed!=was_armed) { std::cout<<(armed?"ARMED":"DISARMED: decelerating to hold")<<std::endl; was_armed=armed; }
      if(stopping&&!was_stopping) { std::cout<<"STOPPING: input/robot timing, gripper fault, duration or exit request"<<std::endl; was_stopping=true; }
      if(data_.refused.exchange(false)) std::cout<<"Arm refused: release all motion controls and A/B; wait for rest, then press Y again"<<std::endl;
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
    if(opt.mode=="validate-config") { std::cout<<"Configuration valid: delta control"<<std::endl; return 0; }
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
        std::cout<<" buttons_mask="<<s.buttons<<" mapped [X Y Z Rx Ry Rz]: ";
        for(double x:map_input(config,s)) std::cout<<x<<" ";
        std::cout<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } while(!ending());
      return 0;
    }
    std::unique_ptr<franka::Robot> robot;
    Pose initial=identity_pose(); initial[12]=.5; initial[14]=.4;
    if(opt.mode!="simulate") {
      robot=std::make_unique<franka::Robot>(opt.ip);
      const auto state=robot->readOnce(); initial=state.O_T_EE;
      if(!valid_pose(initial)) throw std::runtime_error("Robot supplied an invalid TCP transform");
      pose("Current configured TCP (O_T_EE)",initial);
      if(opt.mode=="read-state") return 0;
    } else pose("Synthetic initial TCP",initial);
    Reader reader(config); std::cout<<reader.description<<std::endl;
    Delta delta(config,initial);
    GripperChannel channel;
    std::unique_ptr<GripperWorker> gripper;
    if(opt.mode=="execute"&&config.gripper) gripper=std::make_unique<GripperWorker>(opt.ip,config,channel);
    Telemetry telemetry; for(size_t i=0;i<16;++i) telemetry.target[i].store(initial[i]);
    Reporter reporter(telemetry,channel);
    uint32_t sequence=0;
    Input last_input=reader.input.load();
    auto tick=[&](double dt,bool feedback_at_rest=true) {
      const auto input=reader.input.load();
      // A brief seqlock collision keeps the preceding snapshot and its original
      // timestamp. A stalled producer still expires at the configured deadline.
      if(input.stamp>0) last_input=input;
      const auto s=delta.update(last_input,steady_time(),dt,channel.pause.load()||channel.pending(),ending()||channel.fault.load());
      channel.authorized_epoch.store(s.armed?s.epoch:0);
      channel.at_rest.store(s.stopped&&feedback_at_rest);
      if(s.grip!=Grip::None) {
        if(gripper) channel.publish(s.epoch,++sequence,s.grip);
        // Preview/simulation expose intent without opening a Gripper connection.
      }
      telemetry.store(s); return s;
    };
    std::cout<<"Mode: "<<opt.mode<<"; delta "<<config.speed*1000<<" mm/s, "<<config.rotation*180/kPi
      <<" deg/s; DISARMED; Y toggles arm/disarm; A grasp, B fully open"<<std::endl;
    if(opt.mode=="execute") {
      bool first=true;
      double stationary_time=0;
      try {
        robot->control([&](const franka::RobotState& state,franka::Duration period)->franka::CartesianPose {
          if(first) {
            // Start at the robot's current commanded pose to preserve FCI startup
            // continuity, rather than commanding a measured/commanded pose jump.
            delta.seed_command_pose(state.O_T_EE_c); first=false;
          }
          bool stationary=true;
          for(size_t i=0;i<6;++i)
            stationary=stationary&&std::abs(state.O_dP_EE_c[i])<(i<3?.0001:.001);
          for(double dq:state.dq) stationary=stationary&&std::abs(dq)<.01;
          stationary_time=stationary?stationary_time+period.toSec():0;
          const auto s=tick(period.toSec(),stationary_time>=.10);
          const franka::CartesianPose command(s.target);
          return s.shutdown&&channel.at_rest.load()?franka::MotionFinished(command):command;
        },franka::ControllerMode::kCartesianImpedance,true,config.cutoff);
      } catch(...) { channel.authorized_epoch.store(0); throw; }
    } else if(opt.mode=="preview") {
      double previous=0;
      robot->read([&](const franka::RobotState& state) {
        const double now=state.time.toSec(); const double dt=previous==0?0:now-previous; previous=now;
        const auto s=tick(dt); return !(s.shutdown&&s.stopped);
      });
    } else {
      auto previous=steady_time();
      while(true) {
        const auto now=steady_time(); const auto s=tick(now-previous); previous=now;
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
