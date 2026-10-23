#pragma once
#include "franka_gamepad_teleop/core.hpp"
#include <SDL.h>
#include <thread>

namespace gamepad {
class Device {
 public:
  explicit Device(const Config&);
  ~Device();
  Device(const Device&)=delete;
  Device& operator=(const Device&)=delete;
  Input sample();
  std::string description() const;
 private:
  SDL_GameController* controller_=nullptr;
};
class Reader {
 public:
  explicit Reader(const Config&);
  ~Reader();
  InputBuffer input;
  std::string description;
 private:
  std::atomic<bool> stop_{false};
  std::thread thread_;
};
}  // namespace gamepad
