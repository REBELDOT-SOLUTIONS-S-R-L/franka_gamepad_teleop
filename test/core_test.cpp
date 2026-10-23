#include "franka_gamepad_teleop/core.hpp"
#include "franka_gamepad_teleop/input.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

using namespace gamepad;
namespace {
void require(bool yes,const char* message) { if(!yes) throw std::runtime_error(message); }
void near(double actual,double wanted,double tolerance=1e-9) { require(std::abs(actual-wanted)<tolerance,"numeric mismatch"); }
struct Fixture {
  Config config; Delta delta; Input input; double now=100; Step last;
  Fixture(const Config& c,const Pose& pose=identity_pose()):config(c),delta(c,pose) {
    input.connected=true; tick();
  }
  Step tick(double dt=.001,bool pause=false,bool stop=false) {
    now+=dt; input.stamp=now; return last=delta.update(input,now,dt,pause,stop);
  }
  void press(int button) { input.buttons|=1U<<button; tick(); input.buttons&=~(1U<<button); tick(); }
  void arm() { press(config.arm); require(last.armed,"failed to arm"); }
  void run(int ticks) { for(int i=0;i<ticks;++i) tick(); }
};
}
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  const auto config=load_config(argv[1]);
  int count=0;
  auto test=[&](const std::string& name,const std::function<void()>& f) {
    try { f(); ++count; std::cout<<"PASS "<<name<<"\n"; }
    catch(const std::exception& e) { std::cerr<<"FAIL "<<name<<": "<<e.what()<<"\n"; std::exit(1); }
  };
  test("configured rates",[&] { near(config.speed,.02); near(config.rotation,20*kPi/180); near(config.force,30); });
  test("base translation and TCP rotation/local-Z binding signs",[&] {
    Input s; s.axes[1]=-1; near(map_input(config,s)[0],1);
    s={}; s.axes[0]=1; near(map_input(config,s)[1],-1);
    s={}; s.axes[4]=1; near(map_input(config,s)[2],1);
    s={}; s.axes[5]=1; near(map_input(config,s)[2],-1);
    s={}; s.axes[2]=1; near(map_input(config,s)[3],1);
    s={}; s.axes[3]=-1; near(map_input(config,s)[4],1);
    s={}; s.buttons=1U<<10; near(map_input(config,s)[5],1);
    s.buttons=1U<<9; near(map_input(config,s)[5],-1);
    s={}; s.buttons=1U<<button_index("dpad_up"); near(map_input(config,s)[6],1);
    s.buttons=1U<<button_index("dpad_down"); near(map_input(config,s)[6],-1);
    s.buttons|=1U<<button_index("dpad_up"); near(map_input(config,s)[6],0);
  });
  test("proportional deadzones and cancelling triggers",[&] {
    near(deadzone(.12,.12,false),0); near(deadzone(.56,.12,false),.5);
    near(deadzone(-.56,.12,false),-.5); near(deadzone(.52,.04,true),.5);
    Input s; s.axes[4]=s.axes[5]=1; near(map_input(config,s)[2],0);
  });
  test("starts disarmed; a held Y is not a press",[&] {
    Delta d(config,identity_pose()); Input s; s.connected=true; s.stamp=1; s.buttons=1U<<config.arm;
    require(!d.update(s,1,.001).armed,"startup Y armed"); s.stamp=1.1;
    require(!d.update(s,1.1,.001).armed,"held Y armed");
  });
  test("Y toggles only on rising edges",[&] {
    Fixture f(config); f.arm(); f.input.buttons=1U<<config.arm;
    require(!f.tick().armed,"Y did not disarm"); f.run(100);
    require(!f.last.armed,"held Y rearmed");
  });
  test("arming rejects deflected and opposing controls",[&] {
    for(int mode=0;mode<5;++mode) {
      Fixture f(config);
      if(mode==0) f.input.axes[0]=1;
      if(mode==1) f.input.axes[4]=f.input.axes[5]=1;
      if(mode==2) f.input.buttons=(1U<<9)|(1U<<10);
      if(mode==3) f.input.buttons=1U<<button_index("dpad_up");
      if(mode==4) f.input.buttons=(1U<<button_index("dpad_up"))|(1U<<button_index("dpad_down"));
      f.input.buttons|=1U<<config.arm;
      require(f.tick().arm_refused&&!f.last.armed,"non-neutral input armed");
    }
  });
  test("arming rejects held gripper buttons",[&] {
    Fixture f(config); f.input.buttons=(1U<<config.arm)|(1U<<config.close);
    require(f.tick().arm_refused&&!f.last.armed,"gripper held while arming");
  });
  test("translation speed vector cap and 20 mm per steady second",[&] {
    Fixture f(config); f.arm(); f.input.axes[1]=-1; f.run(1000);
    const auto before=f.last.target; f.run(1000); near(f.last.target[12]-before[12],.020,1e-10);
    Fixture diagonal(config); diagonal.arm(); diagonal.input.axes[0]=1; diagonal.input.axes[1]=-1;
    diagonal.run(1000); const auto a=diagonal.last.target; diagonal.run(1000);
    near(std::hypot(diagonal.last.target[12]-a[12],diagonal.last.target[13]-a[13]),.020,1e-10);
  });
  test("half deflection produces half rate",[&] {
    Fixture f(config); f.arm(); f.input.axes[1]=-.56; f.run(1000); const double x=f.last.target[12]; f.run(1000);
    near(f.last.target[12]-x,.010,1e-10);
  });
  test("base translation is independent of TCP orientation",[&] {
    auto initial=identity_pose(); Eigen::Map<Eigen::Matrix4d> t(initial.data());
    t.topLeftCorner<3,3>()=(Eigen::AngleAxisd(kPi/2,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(kPi/3,Eigen::Vector3d::UnitY())).toRotationMatrix();
    Fixture f(config,initial); f.arm(); f.input.axes[1]=-1; f.run(1000);
    require(f.last.target[12]>.015,"base X not advanced"); near(f.last.target[13],0);
    Fixture z(config,initial); z.arm(); z.input.axes[4]=1; z.run(1000);
    near(z.last.target[12],0); near(z.last.target[13],0);
    require(z.last.target[14]>.015,"trigger Z unexpectedly used the TCP frame");
  });
  test("all rotations use TCP-local axes and preserve TCP position",[&] {
    auto initial=identity_pose(); initial[12]=.53; initial[13]=-.2; initial[14]=.3;
    Eigen::Map<Eigen::Matrix4d> t(initial.data());
    const Eigen::Matrix3d start=(Eigen::AngleAxisd(kPi/2,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(kPi/3,Eigen::Vector3d::UnitY())).toRotationMatrix();
    t.topLeftCorner<3,3>()=start;
    for(int axis=0;axis<3;++axis) {
      Fixture f(config,initial); f.arm();
      if(axis==0) f.input.axes[2]=1;
      if(axis==1) f.input.axes[3]=-1;
      if(axis==2) f.input.buttons=1U<<button_index("right_shoulder");
      f.run(1000); const auto before=f.last.target; f.run(1000);
      Eigen::Map<const Eigen::Matrix4d> first(before.data()),last(f.last.target.data());
      const Eigen::Matrix3d expected=first.topLeftCorner<3,3>()*
        Eigen::AngleAxisd(config.rotation,Eigen::Vector3d::Unit(axis)).toRotationMatrix();
      require((last.topLeftCorner<3,3>()-expected).norm()<1e-9,"rotation was not about its TCP-local axis");
      near(f.last.target[12],initial[12]); near(f.last.target[13],initial[13]); near(f.last.target[14],initial[14]);
      require(valid_pose(f.last.target),"invalid integrated transform");
    }
  });
  test("D-pad translates along tilted TCP local Z in both directions",[&] {
    auto initial=identity_pose(); Eigen::Map<Eigen::Matrix4d> t(initial.data());
    t.topLeftCorner<3,3>()=Eigen::AngleAxisd(kPi/2,Eigen::Vector3d::UnitY()).toRotationMatrix();
    for(int sign:{-1,1}) {
      Fixture f(config,initial); f.arm();
      f.input.buttons=1U<<button_index(sign>0?"dpad_up":"dpad_down");
      f.run(1000); const auto before=f.last.target; f.run(1000);
      near(f.last.target[12]-before[12],sign*.020,1e-10);
      near(f.last.target[13],0); near(f.last.target[14],0);
    }
  });
  test("combined base and TCP translation retains the vector speed limit",[&] {
    for(int tilted=0;tilted<2;++tilted) {
      auto initial=identity_pose(); Eigen::Map<Eigen::Matrix4d> t(initial.data());
      if(tilted) t.topLeftCorner<3,3>()=Eigen::AngleAxisd(kPi/2,Eigen::Vector3d::UnitY()).toRotationMatrix();
      Fixture f(config,initial); f.arm(); f.input.axes[4]=1;
      f.input.buttons=1U<<button_index("dpad_up");
      f.run(1000); const auto before=f.last.target; f.run(1000);
      Eigen::Map<const Eigen::Matrix4d> a(before.data()),b(f.last.target.data());
      Eigen::Vector3d direction=t.topLeftCorner<3,3>().col(2)+Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d displacement=b.topRightCorner<3,1>()-a.topRightCorner<3,1>();
      near(displacement.norm(),.020,1e-10);
      require((displacement-.020*direction.normalized()).norm()<1e-9,"mixed-frame translation direction incorrect");
    }
  });
  test("D-pad local Z follows the changed TCP orientation",[&] {
    Fixture f(config); f.arm(); f.input.axes[3]=-1;
    f.input.buttons=1U<<button_index("dpad_up"); f.run(2000);
    f.input.axes[3]=0; f.run(1000);
    const auto before=f.last.target; f.run(1000);
    Eigen::Map<const Eigen::Matrix4d> a(before.data()),b(f.last.target.data());
    const Eigen::Vector3d displacement=b.topRightCorner<3,1>()-a.topRightCorner<3,1>();
    const Eigen::Vector3d expected=a.topLeftCorner<3,3>().col(2)*.020;
    require((displacement-expected).norm()<1e-9,"D-pad translation used the initial or base Z axis");
  });
  test("local Z bindings are configurable and optional for older profiles",[&] {
    const auto tmp=std::filesystem::temp_directory_path()/("gamepad-local-z-test-"+std::to_string(getpid())+".yaml");
    auto node=YAML::LoadFile(argv[1]);
    node["bindings"]["translation_tcp_z"][0]["source"]="dpad_left";
    { std::ofstream out(tmp); out<<node; }
    const auto customized=load_config(tmp);
    Input s; s.buttons=1U<<button_index("dpad_left"); near(map_input(customized,s)[6],1);
    s.buttons=1U<<button_index("dpad_up"); near(map_input(customized,s)[6],0);
    node["bindings"].remove("translation_tcp_z");
    { std::ofstream out(tmp); out<<node; }
    const auto legacy=load_config(tmp); near(map_input(legacy,s)[6],0);
    std::filesystem::remove(tmp);
  });
  test("disarm decelerates continuously then holds",[&] {
    Fixture f(config); f.arm(); f.input.axes[1]=-1; f.run(1000);
    const double before=f.last.target[12]; f.press(config.arm);
    require(!f.last.armed&&!f.last.stopped,"disarm did not start deceleration");
    f.run(600); require(f.last.stopped,"did not reach rest");
    require(f.last.target[12]>before,"disarm jumped instead of decelerating");
    const auto held=f.last.target; f.run(1000); require(f.last.target==held,"disarmed pose changed");
  });
  test("stale input latches stop and cannot auto-rearm",[&] {
    Fixture f(config); f.arm(); f.input.axes[1]=-1; f.run(1000);
    f.now+=config.timeout+.01; auto s=f.delta.update(f.input,f.now,.001);
    require(s.shutdown&&!s.armed,"stale input did not stop");
    f.input.axes={}; f.run(600); f.press(config.arm);
    require(f.last.shutdown&&!f.last.armed&&f.last.stopped,"stale-input session rearmed");
  });
  test("disconnect, nonfinite input and timing gap stop",[&] {
    for(int mode=0;mode<3;++mode) {
      Fixture f(config); f.arm();
      if(mode==0) f.input.connected=false;
      if(mode==1) f.input.axes[0]=std::nan("");
      const auto s=f.tick(mode==2?.021:.001);
      require(s.shutdown&&!s.armed,"invalid input/timing did not stop");
    }
  });
  test("gripper edges, open priority, pause and no disarmed commands",[&] {
    Fixture f(config); f.press(config.close); require(f.last.grip==Grip::None,"disarmed grip emitted");
    f.arm(); f.input.buttons=(1U<<config.close)|(1U<<config.open);
    require(f.tick().grip==Grip::Open,"open did not win simultaneous input");
    require(f.tick().grip==Grip::None,"held grip repeated");
    f.input.buttons=0; f.input.axes[1]=-1;
    for(int i=0;i<1000;++i) f.tick(.001,true);
    near(f.last.target[12],0);
  });
  test("gripper epoch fences cancelled and replaced actions",[&] {
    GripperChannel ch; ch.authorized_epoch.store(1); ch.publish(1,1,Grip::Close); const auto first=ch.request.load();
    require(ch.permits(first),"new request refused");
    ch.authorized_epoch.store(0); require(!ch.permits(first),"disarmed request permitted");
    ch.authorized_epoch.store(2); require(!ch.permits(first),"old epoch survived rearm");
    ch.publish(2,2,Grip::Open); const auto second=ch.request.load(); require(ch.permits(second),"new session refused");
    ch.publish(2,3,Grip::Close); require(!ch.permits(second),"superseded command permitted");
    ch.completed.store(second); ch.pause.store(false);
    require(ch.pending(),"finishing an older operation lost the replacement pause");
    ch.fault.store(true); require(!ch.permits(ch.request.load()),"faulted gripper request permitted");
  });
  test("no imposed application workspace box",[&] {
    auto initial=identity_pose(); initial[12]=-.3; initial[13]=.8; initial[14]=1.2;
    Fixture f(config,initial); f.arm(); f.input.axes[1]=-1; f.run(1000);
    require(f.last.target[12]>initial[12],"unexpected workspace constraint");
  });
  test("invalid initial pose rejected",[&] {
    auto bad=identity_pose(); bad[0]=2; bool threw=false;
    try { Delta d(config,bad); } catch(...) { threw=true; }
    require(threw,"invalid pose accepted");
  });
  test("atomic input snapshots are coherent",[&] {
    InputBuffer buffer; std::atomic<bool> done{false};
    std::thread writer([&] {
      for(int i=1;i<=10000;++i) { Input s; s.axes.fill(i%2?.25:-.25); s.stamp=i; s.buttons=i%2?1:2; s.connected=true; buffer.store(s); }
      done.store(true);
    });
    bool coherent=true;
    do {
      const auto s=buffer.load(); if(s.stamp>0) {
        for(double a:s.axes) coherent=coherent&&(a==s.axes[0]);
        coherent=coherent&&(s.buttons==(s.axes[0]>.0?1U:2U));
      }
    } while(!done.load());
    writer.join(); require(coherent,"torn snapshot read");
  });
  test("invalid YAML mode, binding, rates and duplicate actions rejected",[&] {
    const auto tmp=std::filesystem::temp_directory_path()/("gamepad-config-test-"+std::to_string(getpid())+".yaml");
    for(int mode=0;mode<5;++mode) {
      auto node=YAML::LoadFile(argv[1]);
      if(mode==0) node["mode"]="force";
      if(mode==1) node["bindings"]["translation_x"][0]["source"]="typo";
      if(mode==2) node["motion"]["max_translation_m_s"]=-1;
      if(mode==3) node["bindings"]["gripper_open"]="a";
      if(mode==4) node["motion"]["typo"]=1;
      { std::ofstream out(tmp); out<<node; }
      bool threw=false; try { load_config(tmp); } catch(...) { threw=true; }
      require(threw,"invalid YAML accepted");
    }
    std::filesystem::remove(tmp);
  });
  test("SDL virtual gamepad reads mapped axes and disconnect",[&] {
    require(SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER)==0,"SDL init failed");
    const int index=SDL_JoystickAttachVirtual(SDL_JOYSTICK_TYPE_GAMECONTROLLER,6,15,0);
    require(index>=0,"cannot attach virtual gamepad");
    SDL_Joystick* joystick=SDL_JoystickOpen(index);
    require(joystick!=nullptr,"cannot open virtual joystick");
    Config c=config; c.name="Virtual Controller";
    {
      Device device(c);
      SDL_JoystickSetVirtualAxis(joystick,0,32767);
      SDL_JoystickSetVirtualAxis(joystick,1,-32768);
      SDL_JoystickSetVirtualButton(joystick,SDL_CONTROLLER_BUTTON_Y,1);
      SDL_JoystickSetVirtualButton(joystick,SDL_CONTROLLER_BUTTON_DPAD_UP,1);
      const auto input=device.sample();
      require(input.connected,"virtual pad missing"); near(input.axes[0],1); near(input.axes[1],-1);
      require(input.buttons&(1U<<c.arm),"virtual Y missing");
      near(map_input(c,input)[6],1);
      SDL_JoystickDetachVirtual(index);
      require(!device.sample().connected,"disconnect was not reported");
    }
    SDL_JoystickClose(joystick); SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
  });
  std::cout<<count<<" tests passed\n";
}
