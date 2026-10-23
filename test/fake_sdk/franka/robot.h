#pragma once
// Test-only API simulator. It opens no sockets and never links libfranka.
#include <SDL.h>
#include <array>
#include <chrono>
#include <csignal>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>
namespace franka {
struct Duration { double seconds=0; double toSec() const { return seconds; } };
enum class ControllerMode { kCartesianImpedance };
enum class Frame { kEndEffector };
enum class RobotMode { kMove };
struct CartesianPose { std::array<double,16> O_T_EE; bool motion_finished=false; explicit CartesianPose(const std::array<double,16>& p):O_T_EE(p) {} };
struct CartesianVelocities { std::array<double,6> O_dP_EE; bool motion_finished=false; explicit CartesianVelocities(const std::array<double,6>& p):O_dP_EE(p) {} };
struct Torques { std::array<double,7> tau_J; bool motion_finished=false; explicit Torques(const std::array<double,7>& p):tau_J(p) {} };
template<class T> T MotionFinished(T command) { command.motion_finished=true; return command; }
struct RobotState {
  std::array<double,16> O_T_EE{},O_T_EE_c{},O_T_EE_d{},F_T_EE{},F_T_NE{},NE_T_EE{},EE_T_K{};
  std::array<double,7> q{0,0,0,-1.5,0,2,0},q_d=q,dq{},tau_ext_hat_filtered{},tau_J_d{};
  std::array<double,6> O_dP_EE_c{},O_ddP_EE_c{},O_F_ext_hat_K{};
  std::array<double,3> F_x_Cee{},F_x_Cload{};
  double m_ee=.73,m_load=0,control_command_success_rate=1;
  Duration time; RobotMode robot_mode=RobotMode::kMove;
};
namespace api_test {
inline SDL_Joystick* joystick=nullptr;
inline double time=0;
inline bool interrupted=false;
inline int model_loads=0;
inline std::vector<std::string> backends;
inline double maximum_velocity=0,maximum_torque=0,maximum_pose_jump=0;
inline void input() {
  const auto down=[](double a,double b){return time>=a&&time<b;};
  const bool x=down(.15,.19)||down(1.15,1.19)||down(2.35,2.39);
  const bool y=down(.35,.39)||down(.95,.99)||down(1.35,1.39)||down(2.1,2.14);
  SDL_JoystickSetVirtualButton(joystick,SDL_CONTROLLER_BUTTON_X,x);
  SDL_JoystickSetVirtualButton(joystick,SDL_CONTROLLER_BUTTON_Y,y);
  const bool move=down(.5,.65)||down(1.5,1.8);
  SDL_JoystickSetVirtualAxis(joystick,SDL_CONTROLLER_AXIS_LEFTY,move?-32768:0);
  if(time>2.7&&!interrupted) { interrupted=true;std::raise(SIGINT); }
}
}
class Model {
 public:
  std::array<double,42> zeroJacobian(Frame,const RobotState&) const {
    std::array<double,42> j{}; for(size_t i=0;i<6;++i) j[i+6*i]=1;return j;
  }
  std::array<double,7> coriolis(const RobotState&) const { return {}; }
};
class Robot {
 public:
  explicit Robot(const std::string& ip) {
    if(ip!="fake-only") throw std::runtime_error("Fake SDK accepts only fake-only; no network access");
    for(auto* p:{&state_.O_T_EE,&state_.O_T_EE_c,&state_.O_T_EE_d,&state_.F_T_EE,&state_.F_T_NE,&state_.NE_T_EE,&state_.EE_T_K})
      for(size_t i:{0U,5U,10U,15U}) (*p)[i]=1;
  }
  RobotState readOnce() { return state_; }
  Model loadModel() { ++api_test::model_loads;return {}; }
  template<class Callback> void control(Callback cb,ControllerMode,bool,double) { run(cb); }
  template<class Callback> void control(Callback cb,bool,double) { run(cb); }
  template<class Callback> void read(Callback cb) { while(cb(state_)) {state_.time.seconds+=.001;} }
 private:
  RobotState state_;
  template<class Callback> void run(Callback cb) {
    using Command=std::invoke_result_t<Callback,const RobotState&,Duration>;
    api_test::backends.push_back(std::is_same_v<Command,CartesianPose>?"pose":std::is_same_v<Command,CartesianVelocities>?"velocity":"torque");
    bool first=true;
    for(int iterations=0;iterations<15000;++iterations) {
      const double dt=first?0:.001;
      api_test::time+=dt;state_.time.seconds=api_test::time;api_test::input();
      const auto command=cb(state_,Duration{dt});
      if constexpr(std::is_same_v<Command,CartesianPose>) {
        if(first) {
          double sum=0;for(size_t i:{12U,13U,14U}) sum+=std::pow(command.O_T_EE[i]-state_.O_T_EE[i],2);
          api_test::maximum_pose_jump=std::max(api_test::maximum_pose_jump,std::sqrt(sum));
        }
        state_.O_T_EE=state_.O_T_EE_c=state_.O_T_EE_d=command.O_T_EE;
        state_.O_dP_EE_c.fill(0);
      } else if constexpr(std::is_same_v<Command,CartesianVelocities>) {
        state_.O_dP_EE_c=command.O_dP_EE;
        for(size_t i=0;i<3;++i) {
          api_test::maximum_velocity=std::max(api_test::maximum_velocity,std::abs(command.O_dP_EE[i]));
          state_.O_T_EE[12+i]+=command.O_dP_EE[i]*dt;
        }
        state_.O_T_EE_c=state_.O_T_EE_d=state_.O_T_EE;
      } else {
        for(double value:command.tau_J) api_test::maximum_torque=std::max(api_test::maximum_torque,std::abs(value));
        state_.tau_J_d=command.tau_J;
      }
      if(command.motion_finished) return;
      first=false;std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("Fake SDK controller failed to finish");
  }
};
}  // namespace franka
