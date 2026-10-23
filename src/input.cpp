#include "franka_gamepad_teleop/input.hpp"
#include <future>
#include <stdexcept>
#include <vector>

namespace gamepad {
Device::Device(const Config& config) {
  SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS,"1");
  if(SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER)!=0) throw std::runtime_error(SDL_GetError());
  std::vector<int> matches;
  for(int i=0;i<SDL_NumJoysticks();++i) {
    const char* name=SDL_JoystickNameForIndex(i);
    if(SDL_IsGameController(i)&&name&&std::string(name).find(config.name)!=std::string::npos) matches.push_back(i);
  }
  if(matches.size()!=1) {
    SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
    throw std::runtime_error("Expected exactly one recognized gamepad matching '"+config.name+"'; found "+std::to_string(matches.size()));
  }
  controller_=SDL_GameControllerOpen(matches[0]);
  if(!controller_) { const std::string error=SDL_GetError(); SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER); throw std::runtime_error(error); }
  for(int axis=0;axis<6;++axis) {
    if(!SDL_GameControllerHasAxis(controller_,static_cast<SDL_GameControllerAxis>(axis))) {
      SDL_GameControllerClose(controller_); SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
      throw std::runtime_error("Selected gamepad lacks a required stick/trigger axis");
    }
  }
  for(int button:{config.arm,config.close,config.open,config.cycle}) {
    if(!SDL_GameControllerHasButton(controller_,static_cast<SDL_GameControllerButton>(button))) {
      SDL_GameControllerClose(controller_); SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
      throw std::runtime_error("Selected gamepad lacks a configured action button");
    }
  }
}
Device::~Device() { SDL_GameControllerClose(controller_); SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER); }
Input Device::sample() {
  SDL_PumpEvents();
  SDL_GameControllerUpdate();
  SDL_FlushEvents(SDL_FIRSTEVENT,SDL_LASTEVENT);
  Input s;
  s.connected=SDL_GameControllerGetAttached(controller_)==SDL_TRUE;
  if(s.connected) {
    for(int i=0;i<6;++i) {
      const auto value=SDL_GameControllerGetAxis(controller_,static_cast<SDL_GameControllerAxis>(i));
      s.axes[i]=i>=4?std::max(0.,value/32767.):value/(value<0?32768.:32767.);
    }
    for(int i=0;i<15;++i)
      if(SDL_GameControllerGetButton(controller_,static_cast<SDL_GameControllerButton>(i))) s.buttons|=1U<<i;
  }
  s.stamp=steady_time();
  return s;
}
std::string Device::description() const {
  char* mapping=SDL_GameControllerMapping(controller_);
  const std::string out=std::string(SDL_GameControllerName(controller_))+"\nSDL mapping: "+(mapping?mapping:"unavailable");
  SDL_free(mapping); return out;
}
Reader::Reader(const Config& config) {
  std::promise<std::string> ready;
  auto future=ready.get_future();
  thread_=std::thread([this,config,ready=std::move(ready)]() mutable {
    bool started=false;
    try {
      Device device(config);
      input.store(device.sample());
      ready.set_value(device.description()); started=true;
      auto next=std::chrono::steady_clock::now();
      while(!stop_.load()) {
        const auto s=device.sample(); input.store(s);
        if(!s.connected) return;
        next+=std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1/config.poll_hz));
        std::this_thread::sleep_until(next);
      }
    } catch(...) {
      input.store({});
      if(!started) ready.set_exception(std::current_exception());
    }
  });
  try { description=future.get(); }
  catch(...) { stop_.store(true); thread_.join(); throw; }
}
Reader::~Reader() { stop_.store(true); if(thread_.joinable()) thread_.join(); }
}  // namespace gamepad
