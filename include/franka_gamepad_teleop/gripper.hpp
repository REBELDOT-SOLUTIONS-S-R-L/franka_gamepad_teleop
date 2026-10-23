#pragma once
#include "franka_gamepad_teleop/core.hpp"
#include <thread>

namespace gamepad {
// Network operations and logging are confined to this non-realtime worker.
class GripperWorker {
 public:
  GripperWorker(const std::string& ip,const Config&,GripperChannel&);
  ~GripperWorker();
 private:
  std::atomic<bool> stop_{false};
  std::thread thread_;
};
}  // namespace gamepad
