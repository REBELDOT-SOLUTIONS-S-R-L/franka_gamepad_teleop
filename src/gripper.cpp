#include "franka_gamepad_teleop/gripper.hpp"
#include <franka/gripper.h>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <stdexcept>

namespace gamepad {
GripperWorker::GripperWorker(const std::string& ip,const Config& c,GripperChannel& channel) {
  std::promise<void> ready; auto future=ready.get_future();
  thread_=std::thread([this,ip,c,&channel,ready=std::move(ready)]() mutable {
    bool started=false;
    try {
      franka::Gripper hand(ip);
      const auto initial=hand.readOnce();
      const double maximum=initial.max_width;
      if(!std::isfinite(maximum)||maximum<=0||maximum>.081)
        throw std::runtime_error("Franka Hand has no valid calibrated maximum width; home it separately before launch");
      std::cout<<"Gripper: width="<<initial.width*1000<<" mm; maximum="<<maximum*1000<<" mm\n";
      ready.set_value(); started=true;
      std::future<std::string> operation;
      uint64_t seen=0, active=0;
      while(true) {
        const bool ending=stop_.load();
        const uint64_t token=channel.request.load();
        if(operation.valid()) {
          if(ending||!channel.permits(active)) {
            // Repeated stops fence even a task preempted immediately before its
            // network call. Do not wait for a blocking grasp to finish first.
            try { hand.stop(); }
            catch(const std::exception& error) {
              channel.fault.store(true);
              std::cerr<<"Gripper stop failed: "<<error.what()<<std::endl;
            }
          }
          if(operation.wait_for(std::chrono::milliseconds(0))==std::future_status::ready) {
            try { std::cout<<operation.get()<<std::endl; }
            catch(const std::exception& error) {
              channel.fault.store(true);
              std::cerr<<"Gripper error: "<<error.what()<<std::endl;
            }
            channel.completed.store(active);
            channel.pause.store(false);
          }
        } else if(ending) {
          channel.pause.store(false); break;
        } else if(token!=seen) {
          if(!channel.permits(token)) { seen=token; channel.completed.store(token); channel.pause.store(false); }
          else if(channel.at_rest.load()) {
            seen=active=token;
            channel.pause.store(true);
            operation=std::async(std::launch::async,[&hand,&channel,token,maximum,c] {
              if(!channel.permits(token)) return std::string("Gripper request cancelled before dispatch");
              const bool closing=(token&3U)==static_cast<uint8_t>(Grip::Close);
              const bool ok=closing?hand.grasp(0.,c.gripper_speed,c.force,c.epsilon_inner,c.epsilon_outer)
                :hand.move(maximum,c.gripper_speed);
              if(!channel.permits(token)) return std::string("Gripper operation cancelled");
              const auto state=hand.readOnce();
              if(closing) {
                const bool object=ok&&state.is_grasped&&state.width>.002&&state.width<maximum-.001;
                return object?"GRASP CONFIRMED: width="+std::to_string(state.width*1000)+" mm"
                  :std::string("GRASP: no object confirmed; fingers remain at their resulting width");
              }
              if(!ok) throw std::runtime_error("Gripper opening did not complete");
              return "OPEN: width="+std::to_string(state.width*1000)+" mm";
            });
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    } catch(...) {
      channel.fault.store(true); channel.pause.store(false);
      if(!started) ready.set_exception(std::current_exception());
      else {
        try { throw; } catch(const std::exception& error) { std::cerr<<"Gripper worker failed: "<<error.what()<<std::endl; }
      }
    }
  });
  try { future.get(); }
  catch(...) { stop_.store(true); thread_.join(); throw; }
}
GripperWorker::~GripperWorker() { stop_.store(true); if(thread_.joinable()) thread_.join(); }
}  // namespace gamepad
