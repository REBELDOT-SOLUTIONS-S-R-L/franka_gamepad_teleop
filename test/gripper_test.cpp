#include "franka_gamepad_teleop/gripper.hpp"
#include <franka/gripper.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace gamepad;
namespace fake=gamepad_test_sdk;
namespace {
void require(bool yes,const char* error) { if(!yes) throw std::runtime_error(error); }
void wait_for(const std::function<bool()>& condition) {
  const auto deadline=steady_time()+2;
  while(!condition()) {
    if(steady_time()>deadline) throw std::runtime_error("Timed out waiting for fake gripper worker");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
template<class F> auto locked(F f) { std::lock_guard<std::mutex> lock(fake::mutex); return f(); }
void complete() { locked([] { fake::finish=true; fake::cv.notify_all(); }); }
}
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  const auto c=load_config(argv[1]); int tests=0;
  auto test=[&](const char* name,const std::function<void()>& action) {
    fake::reset();
    try { action(); ++tests; std::cout<<"PASS "<<name<<std::endl; }
    catch(const std::exception& e) { std::cerr<<"FAIL "<<name<<": "<<e.what()<<std::endl; std::exit(1); }
  };
  test("startup has no automatic motion",[&] {
    GripperChannel channel;
    { GripperWorker worker("fake-only",c,channel); }
    require(fake::grasps==0&&fake::moves==0&&fake::stops==0,"startup issued a motion");
  });
  test("grasp waits for arm rest and uses the hand-app parameters",[&] {
    GripperChannel channel; channel.at_rest.store(false); channel.authorized_epoch.store(1);
    {
      GripperWorker worker("fake-only",c,channel); channel.publish(1,1,Grip::Close);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      require(locked([]{return fake::grasps==0;}),"grasp ran before arm rest");
      channel.at_rest.store(true); wait_for([]{return locked([]{return fake::active;});});
      require(channel.pause.load()&&channel.pending(),"arm was not held during grasp");
      complete(); wait_for([&]{return !channel.pending()&&!channel.pause.load();});
    }
    require(fake::grasps==1&&fake::last_force==30&&fake::last_speed==.04,"wrong grasp behavior");
  });
  test("disarm cancels a blocking grasp and does not replay it on rearm",[&] {
    GripperChannel channel; channel.authorized_epoch.store(1);
    {
      GripperWorker worker("fake-only",c,channel); channel.publish(1,1,Grip::Close);
      wait_for([]{return locked([]{return fake::active;});});
      channel.authorized_epoch.store(0);
      wait_for([&]{return channel.completed.load()==channel.request.load();});
      channel.authorized_epoch.store(2); std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    require(fake::stops>=1&&fake::grasps==1&&fake::moves==0,"disarm cancellation/replay failed");
  });
  test("B supersedes a closing grasp and opens to the reported maximum",[&] {
    GripperChannel channel; channel.authorized_epoch.store(1);
    {
      GripperWorker worker("fake-only",c,channel); channel.publish(1,1,Grip::Close);
      wait_for([]{return locked([]{return fake::active;});});
      channel.publish(1,2,Grip::Open);
      wait_for([]{return locked([]{return fake::moves==1;});});
      wait_for([&]{return !channel.pending();});
    }
    require(fake::stops>=1&&fake::last_open==.075,"B did not cancel/open to calibrated maximum");
  });
  test("cancellation fences a task delayed immediately before network dispatch",[&] {
    GripperChannel channel; channel.authorized_epoch.store(1); fake::delay_entry=true;
    {
      GripperWorker worker("fake-only",c,channel); channel.publish(1,1,Grip::Close);
      wait_for([]{return locked([]{return fake::entered;});});
      channel.authorized_epoch.store(0);
      wait_for([]{return locked([]{return fake::stops>=1;});});
      locked([]{fake::delay_entry=false;fake::cv.notify_all();});
      wait_for([&]{return channel.completed.load()==channel.request.load();});
    }
    require(fake::stops>=2&&!fake::active,"late task escaped cancellation");
  });
  test("operation error latches fault and blocks subsequent actions",[&] {
    GripperChannel channel; channel.authorized_epoch.store(1); fake::fail=true;
    {
      GripperWorker worker("fake-only",c,channel); channel.publish(1,1,Grip::Close);
      wait_for([&]{return channel.fault.load();});
      channel.publish(1,2,Grip::Open); std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    require(fake::moves==0,"fault allowed another gripper operation");
  });
  std::cout<<tests<<" threaded gripper tests passed"<<std::endl;
}
