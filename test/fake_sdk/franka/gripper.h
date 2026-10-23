#pragma once
// Test-only SDK stand-in. This target never links libfranka or opens sockets.
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <string>

namespace gamepad_test_sdk {
struct State { double width=.075,max_width=.075; bool is_grasped=false; };
inline std::mutex mutex;
inline std::condition_variable cv;
inline State state;
inline int grasps=0,moves=0,stops=0;
inline bool active=false,cancelled=false,finish=false,delay_entry=false,entered=false,fail=false;
inline double last_open=0,last_force=0,last_speed=0;
inline void reset() {
  std::lock_guard<std::mutex> lock(mutex);
  state=State{}; grasps=moves=stops=0;
  active=cancelled=finish=delay_entry=entered=fail=false;
  last_open=last_force=last_speed=0;
}
class Gripper {
 public:
  explicit Gripper(const std::string&) {}
  State readOnce() const { std::lock_guard<std::mutex> lock(mutex); return state; }
  bool grasp(double width,double speed,double force,double inner,double outer) const {
    std::unique_lock<std::mutex> lock(mutex);
    entered=true; cv.notify_all();
    cv.wait(lock,[]{return !delay_entry;});
    if(fail) throw std::runtime_error("injected fake gripper error");
    if(width!=0||inner!=.001||outer!=.08) throw std::runtime_error("unexpected grasp parameters");
    ++grasps; last_force=force; last_speed=speed; active=true; cancelled=false;
    cv.wait(lock,[]{return finish||cancelled;});
    active=false;
    if(!cancelled) { state.width=.03; state.is_grasped=true; }
    return !cancelled;
  }
  bool move(double width,double speed) const {
    std::lock_guard<std::mutex> lock(mutex);
    ++moves;last_open=width;last_speed=speed;state.width=width;state.is_grasped=false;return true;
  }
  bool stop() const {
    std::lock_guard<std::mutex> lock(mutex);
    ++stops; if(active) cancelled=true; cv.notify_all(); return true;
  }
};
}
namespace franka { using Gripper=gamepad_test_sdk::Gripper; }
