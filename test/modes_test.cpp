#include "franka_gamepad_teleop/core.hpp"
#include "franka_gamepad_teleop/force_control.hpp"
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace gamepad;
namespace {
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
void near(double a,double b,double tolerance=1e-8) { require(std::abs(a-b)<tolerance,"numeric mismatch"); }
struct Rig {
  Config config; Teleop app; Input input; Feedback feedback; Step step; double now=10;
  Rig(Config c,const Pose& p=identity_pose()):config(c),app(c,p) {
    input.connected=true; feedback.pose=p; feedback.valid=true; feedback.still=true; tick();
  }
  Step tick(double dt=.001,bool paused=false,bool stop=false) {
    now+=dt; input.stamp=now; return step=app.update(input,now,dt,paused,stop,feedback);
  }
  Step press(int button) {
    input.buttons|=1U<<button; const auto first=tick(); input.buttons&=~(1U<<button); tick(); return first;
  }
  void run(int count=1000) { for(int i=0;i<count;++i) tick(); }
  void arm() { require(press(config.arm).armed,"arming failed"); }
};
ForceState force_state() {
  ForceState s;
  s.q={0,0,0,-1.5,0,2,0};
  Eigen::Map<Eigen::Matrix<double,6,7>> j(s.jacobian.data()); j.setZero(); j.leftCols<6>().setIdentity();
  return s;
}
}
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  const Config defaults=load_config(argv[1]); int count=0;
  auto test=[&](const char* name,const std::function<void()>& action) {
    try { action();++count;std::cout<<"PASS "<<name<<'\n'; }
    catch(const std::exception& e) { std::cerr<<"FAIL "<<name<<": "<<e.what()<<'\n';std::exit(1); }
  };
  test("initial modes and exact cycling order",[&] {
    require(parse_mode("delta")==ControlMode::Delta&&parse_mode("speed")==ControlMode::Speed&&parse_mode("force")==ControlMode::Force,"mode parsing");
    Rig r(defaults);
    for(auto wanted:{ControlMode::Speed,ControlMode::Force,ControlMode::Delta}) {
      const auto s=r.press(r.config.cycle); require(s.mode_changed&&!s.armed&&s.mode==wanted,"wrong cycle sequence");
    }
  });
  test("a held or startup-held X cannot repeat a mode change",[&] {
    Rig r(defaults);r.input.buttons=1U<<r.config.cycle;
    require(r.tick().mode==ControlMode::Speed,"X failed");r.run();require(r.step.mode==ControlMode::Speed,"held X repeated");
    Teleop fresh(defaults,identity_pose());Input i;i.connected=true;i.stamp=1;i.buttons=1U<<defaults.cycle;
    require(fresh.update(i,1,.001,false,false,r.feedback).mode==ControlMode::Delta,"startup X cycled");
  });
  test("armed X is refused and never queued",[&] {
    Rig r(defaults);r.arm();require(r.press(r.config.cycle).mode_refused,"armed X accepted");
    r.press(r.config.arm);r.run();require(r.step.mode==ControlMode::Delta,"refused X replayed later");
  });
  test("actual motion blocks cycling despite zero app commands",[&] {
    Rig r(defaults);r.feedback.still=false;
    require(r.press(r.config.cycle).mode_refused&&r.step.mode==ControlMode::Delta,"moving robot cycled");
  });
  test("deceleration blocks cycling even if feedback says still",[&] {
    Rig r(defaults);r.arm();r.input.axes[1]=-1;r.run();r.press(r.config.arm);
    r.input.axes={};require(r.press(r.config.cycle).mode_refused,"cycle during braking");
  });
  test("nonneutral and opposing motion controls block cycling",[&] {
    for(int variant=0;variant<3;++variant) {
      Rig r(defaults);
      if(variant==0) r.input.axes[0]=1;
      if(variant==1) r.input.axes[4]=r.input.axes[5]=1;
      if(variant==2) r.input.buttons=(1U<<button_index("dpad_up"))|(1U<<button_index("dpad_down"));
      require(r.press(r.config.cycle).mode_refused,"held/cancelling controls cycled");
    }
  });
  test("simultaneous X/Y cannot switch or arm",[&] {
    Rig r(defaults);r.input.buttons=(1U<<r.config.cycle)|(1U<<r.config.arm);
    const auto s=r.tick();require(s.mode_refused&&s.arm_refused&&!s.armed&&s.mode==ControlMode::Delta,"X/Y bypassed interlock");
  });
  test("gripper activity and held gripper action block cycling",[&] {
    Rig r(defaults);r.input.buttons=1U<<r.config.cycle;require(r.tick(.001,true).mode_refused,"busy gripper cycled");
    Rig held(defaults);held.input.buttons=1U<<held.config.close;require(held.press(held.config.cycle).mode_refused,"held gripper button cycled");
  });
  test("backend transition prevents arming and requires a new Y press",[&] {
    Rig r(defaults);r.press(r.config.cycle);r.feedback.backend_ready=false;
    require(r.press(r.config.arm).arm_refused,"armed before backend ready");
    r.feedback.backend_ready=true;r.tick();r.arm();require(r.step.mode==ControlMode::Speed,"mode changed during arm");
  });
  test("stale-input shutdown prevents further cycling",[&] {
    Rig r(defaults);r.now+=1;auto s=r.app.update(r.input,r.now,.001,false,false,r.feedback);
    require(s.shutdown,"did not latch input expiry");r.press(r.config.cycle);require(r.step.mode==ControlMode::Delta&&!r.step.armed,"faulted session cycled");
  });
  test("Speed output is proportional and vector limited",[&] {
    Config c=defaults;c.initial_mode=ControlMode::Speed;Rig r(c);r.arm();r.input.axes[1]=-.56;r.run();near(r.step.velocity[0],c.speed*.5);
    r.input.axes[0]=1;r.input.axes[1]=-1;r.run();
    near(Eigen::Map<const Eigen::Vector3d>(r.step.velocity.data()).norm(),c.speed);
    r.press(c.arm);r.input.axes={};r.run();require(r.step.stopped,"Speed did not stop");near(r.step.velocity[0],0);
  });
  test("Speed uses measured TCP axes for rotation and local Z",[&] {
    Config c=defaults;c.initial_mode=ControlMode::Speed;Rig r(c);
    Eigen::Map<Eigen::Matrix4d> frame(r.feedback.pose.data());
    frame.topLeftCorner<3,3>()=(Eigen::AngleAxisd(kPi/2,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(kPi/2,Eigen::Vector3d::UnitZ())).toRotationMatrix();
    r.arm();r.input.buttons=1U<<button_index("dpad_up");r.input.axes[2]=1;r.run();
    const Eigen::Vector3d expected=frame.topLeftCorner<3,3>().col(2)*c.speed;
    const Eigen::Vector3d angular=frame.topLeftCorner<3,3>().col(0)*c.rotation;
    require((Eigen::Map<const Eigen::Vector3d>(r.step.velocity.data())-expected).norm()<1e-8,"Speed local Z used another frame");
    require((Eigen::Map<const Eigen::Vector3d>(r.step.velocity.data()+3)-angular).norm()<1e-8,"Speed rotation used another frame");
  });
  test("Force maps deflection to bounded 5 N and 0.5 Nm bias",[&] {
    Config c=defaults;c.initial_mode=ControlMode::Force;Rig r(c);r.arm();
    r.input.axes[1]=-1;r.input.axes[0]=1;r.input.axes[2]=1;r.input.buttons=1U<<button_index("right_shoulder");r.run();
    near(Eigen::Map<const Eigen::Vector3d>(r.step.wrench.data()).norm(),5);
    near(Eigen::Map<const Eigen::Vector3d>(r.step.wrench.data()+3).norm(),.5);
    near(Eigen::Map<const Eigen::Matrix<double,6,1>>(r.step.velocity.data()).norm(),0);
    require(r.step.target==identity_pose(),"Force mode integrated a pose target");
  });
  test("Force rotation and D-pad use the current TCP frame",[&] {
    Config c=defaults;c.initial_mode=ControlMode::Force;Rig r(c);
    Eigen::Map<Eigen::Matrix4d> frame(r.feedback.pose.data());frame.topLeftCorner<3,3>()=Eigen::AngleAxisd(kPi/2,Eigen::Vector3d::UnitY()).toRotationMatrix();
    r.arm();r.input.buttons=(1U<<button_index("dpad_up"))|(1U<<button_index("right_shoulder"));r.run();
    near(r.step.wrench[0],5);near(r.step.wrench[2],0);near(r.step.wrench[3],.5);near(r.step.wrench[5],0);
  });
  test("Force neutral release ramps bias down and holds near current position",[&] {
    Config c=defaults;c.initial_mode=ControlMode::Force;Rig r(c);r.arm();r.input.axes[1]=-1;r.run();
    r.feedback.pose[12]=.02;r.input.axes={};const auto s=r.tick();require(s.wrench[0]>0&&s.wrench[0]<5,"force bias jumped");
    r.run();require(r.step.stopped,"force bias did not reach zero");near(r.step.target[12],.02);
  });
  test("sustained force follows driven translation but holds orthogonal position",[&] {
    Config c=defaults;c.initial_mode=ControlMode::Force;Rig r(c);r.arm();r.input.axes[1]=-1;
    r.feedback.pose[12]=.10;r.feedback.pose[13]=.01;r.run();
    near(r.step.target[12],.10);near(r.step.target[13],0);
    auto state=force_state();state.pose=r.feedback.pose;state.q[0]=.10;
    ForceController controller(c.force_control);controller.seed(state);
    const auto output=controller.update(r.step,state,.001);
    require(output.task_wrench[0]>0&&output.task_wrench[1]<0,"driven force or orthogonal hold lost");
  });
  test("continuous contact commands retain force after arbitrary driven travel",[&] {
    Config c=defaults;c.initial_mode=ControlMode::Force;Rig r(c);r.arm();r.input.axes[1]=-1;
    auto state=force_state();ForceController controller(c.force_control);controller.seed(state);
    r.feedback.pose[12]=.15;state.pose=r.feedback.pose;state.q[0]=.15;
    for(int i=0;i<2000;++i) {
      const auto step=r.tick();const auto output=controller.update(step,state,.001);state.previous_torque=output.torque;
      if(i>500) {near(step.wrench[0],5);near(output.task_wrench[0],5);}
    }
    near(r.step.target[12],.15);
  });
  test("sustained moments follow each TCP rotation axis without spring cancellation",[&] {
    for(int axis=0;axis<3;++axis) {
      Config c=defaults;c.initial_mode=ControlMode::Force;Rig r(c);
      Eigen::Map<Eigen::Matrix4d> frame(r.feedback.pose.data());
      const Eigen::Matrix3d start=(Eigen::AngleAxisd(.6,Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitZ())).toRotationMatrix();
      frame.topLeftCorner<3,3>()=start;r.arm();
      if(axis==0) r.input.axes[2]=1;
      if(axis==1) r.input.axes[3]=-1;
      if(axis==2) r.input.buttons=1U<<button_index("right_shoulder");
      const Eigen::Matrix3d current=start*Eigen::AngleAxisd(.8,Eigen::Vector3d::Unit(axis)).toRotationMatrix();
      frame.topLeftCorner<3,3>()=current;r.run();
      Eigen::Map<const Eigen::Matrix4d> target(r.step.target.data());
      require((target.topLeftCorner<3,3>()-current).norm()<1e-8,"angular hold reference stayed at arming orientation");
      const Eigen::Vector3d moment=Eigen::Map<const Eigen::Vector3d>(r.step.wrench.data()+3);
      require((moment-current.col(axis)*.5).norm()<1e-8,"continuous moment lost its TCP frame");
      auto state=force_state();state.pose=r.feedback.pose;ForceController controller(c.force_control);controller.seed(state);
      const auto output=controller.update(r.step,state,.001);
      require((Eigen::Map<const Eigen::Vector3d>(output.task_wrench.data()+3)-moment).norm()<1e-8,"restoring moment cancelled held input");
    }
  });
  test("Force disarm captures the current pose and clears bias",[&] {
    Config c=defaults;c.initial_mode=ControlMode::Force;Rig r(c);r.arm();r.input.axes[1]=-1;r.run();
    r.feedback.pose[12]=.02;r.press(c.arm);r.run();near(r.step.target[12],.02);require(r.step.stopped&&!r.step.armed,"Force disarm failed");
  });
  test("stationarity requires 100 ms of measured and commanded rest",[] {
    RestMonitor r;std::array<double,7> q{};std::array<double,6> v{},cmd{};
    for(int i=0;i<99;++i) require(!r.update(.001,q,v,cmd),"stillness accepted too early");
    require(r.update(.001,q,v,cmd),"rest did not settle");v[0]=.006;require(!r.update(.001,q,v,cmd),"gross measured TCP motion accepted");
    v[0]=0;cmd[3]=.01;require(!r.update(.001,q,v,cmd),"commanded motion accepted");
    q[0]=.01;require(!r.update(.001,q,v,cmd,false),"joint motion accepted");
    q[0]=0;for(int i=0;i<101;++i) r.update(.001,q,v,cmd,false);
    require(r.update(.001,q,v,cmd,false),"torque-mode rest used stale motion-generator velocity");
  });
  test("stationary velocity noise no longer prevents arming",[] {
    RestMonitor monitor;std::array<double,7> q{};std::array<double,6> measured{},commanded{};
    bool ready=false;
    for(int i=0;i<1000;++i) {
      measured[0]=(i%2?1.5:-1.5)*.001;
      measured[1]=(i%3==0?.8:-.4)*.001;
      q[0]=(i%2?1:-1)*.001;
      ready=monitor.update(.001,q,measured,commanded);
    }
    require(ready,"idle estimator noise repeatedly reset the quiet window");
  });
  test("sustained low-speed motion still blocks readiness",[] {
    RestMonitor monitor;std::array<double,7> q{};std::array<double,6> measured{},commanded{};
    measured[0]=.002;
    for(int i=0;i<1000;++i) require(!monitor.update(.001,q,measured,commanded),"sustained 2 mm/s motion accepted as rest");
    measured.fill(0);
    for(int i=0;i<200;++i) monitor.update(.001,q,measured,commanded);
    measured[1]=.006;require(!monitor.update(.001,q,measured,commanded),"gross-motion onset did not clear readiness immediately");
  });
  test("recorded stationary FR3 velocity estimates permit readiness",[&] {
    const auto file=std::filesystem::path(argv[1]).parent_path()/"rest_velocity_noise.csv";
    std::ifstream input(file);require(input.good(),"recorded idle fixture missing");
    std::string line;std::getline(input,line);
    RestMonitor monitor;std::array<double,6> commanded{};
    int samples=0,ready=0;
    while(std::getline(input,line)) {
      std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
      double dt=0;std::array<double,7> dq{};std::array<double,6> velocity{};
      row>>dt;for(auto& value:dq)row>>value;for(auto& value:velocity)row>>value;
      require(!row.fail(),"invalid idle fixture row");
      if(monitor.update(dt,dq,velocity,commanded)) ++ready;
      ++samples;
    }
    require(samples>=400&&ready>samples*.8,"recorded idle measurements still block arming");
  });
  test("arming refusal records the specific stillness condition",[&] {
    Rig r(defaults);r.feedback.still=false;
    const auto refused=r.press(r.config.arm);
    require(refused.arm_refused&&refused.arm_refusal_reason==32,"arming refusal did not identify measured-rest gate");
    r.feedback.still=true;r.arm();
  });
  test("force torque maps the task wrench and compensates Coriolis",[&] {
    auto s=force_state();ForceController controller(defaults.force_control);controller.seed(s);
    Step step;step.target=s.pose;step.wrench[0]=5;s.coriolis[1]=.1;
    ForceOutput out;
    for(int i=0;i<100;++i) {out=controller.update(step,s,.001);s.previous_torque=out.torque;}
    require(out.valid,"force math invalid");near(out.torque[0],5);near(out.torque[1],.1);
  });
  test("restoring and damping wrench oppose deviation and respect total caps",[&] {
    auto s=force_state();ForceController controller(defaults.force_control);controller.seed(s);
    Step step;step.target=s.pose;s.pose[12]=.1;s.dq[1]=.01;
    const auto out=controller.update(step,s,.001);require(out.task_wrench[0]<0&&out.task_wrench[1]<0,"restoring/damping sign wrong");
    require(Eigen::Map<const Eigen::Vector3d>(out.task_wrench.data()).norm()<=5+1e-9,"net task force exceeded cap");
  });
  test("nullspace posture hold does not request Cartesian motion",[&] {
    auto s=force_state();ForceController controller(defaults.force_control);controller.seed(s);s.q[6]=.1;
    Step step;step.target=s.pose;
    const auto out=controller.update(step,s,.001);require(out.torque[6]<0,"nullspace posture was not restored");
    near(Eigen::Map<const Eigen::Matrix<double,6,1>>(out.task_wrench.data()).norm(),0);
  });
  test("joint-limit barrier points inward",[&] {
    auto s=force_state();s.q[6]=defaults.force_control.upper[6]-.02;
    ForceController controller(defaults.force_control);controller.seed(s);Step step;step.target=s.pose;
    require(controller.update(step,s,.001).torque[6]<0,"upper joint barrier pushed outward");
    s.q[6]=defaults.force_control.lower[6]+.02;controller.seed(s);
    require(controller.update(step,s,.001).torque[6]>0,"lower joint barrier pushed outward");
  });
  test("force start continuity, torque limits and rate bounds",[&] {
    auto s=force_state();s.previous_torque.fill(.3);ForceController controller(defaults.force_control);controller.seed(s);
    Step step;step.target=s.pose;step.wrench[0]=100;
    const auto first=controller.update(step,s,0);require(first.torque==s.previous_torque,"first torque jumped");
    const auto next=controller.update(step,s,.001);
    for(size_t i=0;i<7;++i) require(std::abs(next.torque[i]-s.previous_torque[i])<=.20000001,"torque rate exceeded");
    s.coriolis.fill(1000);s.previous_torque.fill(0);
    ForceOutput out;
    for(int k=0;k<1000;++k) {out=controller.update(step,s,.001);s.previous_torque=out.torque;}
    for(size_t i=0;i<7;++i) require(std::abs(out.torque[i])<=defaults.force_control.torque_limits[i]+1e-8,"joint torque cap exceeded");
  });
  test("nonfinite force model data is rejected",[&] {
    auto s=force_state();ForceController controller(defaults.force_control);controller.seed(s);s.jacobian[0]=std::nan("");
    Step step;step.target=s.pose;require(!controller.update(step,s,.001).valid,"nonfinite Jacobian accepted");
  });
  test("free-space plant continues past old 25 mm equilibrium and stops near release",[&] {
    Config c=defaults;c.initial_mode=ControlMode::Force;c.speed=.1;c.force_control.max_linear_speed=c.speed;
    Rig r(c);r.arm();auto s=force_state();ForceController controller(c.force_control);controller.seed(s);
    double x=0,v=0,release_position=0,maximum_speed=0;
    for(int cycle=0;cycle<7000;++cycle) {
      r.input.axes[1]=cycle<4000?-1:0;
      r.feedback.pose[12]=x;r.feedback.still=std::abs(v)<.001;
      s.pose[12]=x;s.q[0]=x;s.dq[0]=v;
      const auto step=r.tick();const auto out=controller.update(step,s,.001);require(out.valid,"plant torque invalid");s.previous_torque=out.torque;
      v+=out.torque[0]*.001;x+=v*.001;
      maximum_speed=std::max(maximum_speed,std::abs(v));
      if(cycle==3999) release_position=x;
    }
    require(release_position>.30,"held force settled near the old arming point");
    require(maximum_speed<=.1001,"nominal free-space damping failed to limit speed");
    require(x>=release_position-.002&&x<release_position+.03,"release pulled back to the old arming point or failed to brake");
    near(v,0,.001);require(r.step.stopped,"released force bias did not reach zero");
  });
  std::cout<<count<<" mode/force tests passed\n";
}
