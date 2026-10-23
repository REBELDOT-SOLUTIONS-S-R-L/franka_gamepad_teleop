#pragma once
#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace gamepad {
constexpr double kPi = 3.14159265358979323846;
using Pose = std::array<double, 16>;  // Column-major O_T_EE, configured EE/TCP.
struct Input {
  std::array<double, 6> axes{};  // SDL leftx/y, rightx/y, left/right trigger.
  uint32_t buttons = 0;
  double stamp = 0;
  bool connected = false;
};
// A bounded seqlock using atomic fields: no data races or locks in the RT reader.
class InputBuffer {
 public:
  void store(const Input& input);
  Input load() const;
 private:
  std::atomic<uint64_t> version_{0};
  std::array<std::atomic<double>, 6> axes_{};
  std::atomic<uint32_t> buttons_{0};
  std::atomic<double> stamp_{0};
  std::atomic<bool> connected_{false};
};
struct Binding { int source; double scale; };  // axes 0..5, buttons 6..20.
enum class ControlMode : uint8_t { Delta=0, Speed=1, Force=2 };
const char* mode_name(ControlMode);
ControlMode parse_mode(const std::string&);
struct ForceSettings {
  double max_force=5, max_torque=.5, force_slew=20, torque_slew=2;
  double translation_stiffness=200, rotation_stiffness=20;
  double translation_damping=28.2842712475, rotation_damping=8.94427191;
  // Derived from the shared motion limits when loading a profile. Damping is
  // increased as needed to bound nominal free-space drift under full bias.
  double max_linear_speed=.020,max_angular_speed=20*kPi/180;
  double nullspace_stiffness=5, nullspace_damping=4.472135955;
  double joint_margin=.10, joint_barrier_stiffness=20, torque_rate=200;
  // FR3 joint limits from franka_description 2.9.0 robots/fr3/joint_limits.yaml.
  std::array<double,7> lower{-2.9007,-1.8361,-2.9007,-3.0770,-2.8763,.4398,-3.0508};
  std::array<double,7> upper{2.9007,1.8361,2.9007,-.1169,2.8763,4.6216,3.0508};
  std::array<double,7> torque_limits{35,35,35,35,8,8,8};
};
struct Config {
  ControlMode initial_mode=ControlMode::Delta;
  ForceSettings force_control;
  std::string name;
  double stick_deadzone = .12, trigger_deadzone = .04, poll_hz = 200, timeout = .10;
  double speed = .020, rotation = 20*kPi/180, acceleration = .05;
  double angular_acceleration = 60*kPi/180, cutoff = 10;
  // Base XYZ, TCP-local rotation XYZ, and additional TCP-local Z translation.
  std::array<std::vector<Binding>, 7> bindings;
  int arm = 3, close = 0, open = 1, cycle = 2;  // Standard SDL buttons.
  bool gripper = true;
  double force = 30, gripper_speed = .04, epsilon_inner = .001, epsilon_outer = .08;
};
Config load_config(const std::string& path);
int source_index(const std::string& name);
int button_index(const std::string& name);
double deadzone(double value, double zone, bool trigger);
std::array<double, 7> map_input(const Config&, const Input&);
Pose identity_pose();
bool valid_pose(const Pose&);
enum class Grip : uint8_t { None = 0, Close = 1, Open = 2 };
struct Step {
  Pose target{};
  ControlMode mode=ControlMode::Delta;
  std::array<double,6> velocity{},wrench{};  // Base-frame m/s, rad/s, N, Nm.
  Grip grip = Grip::None;
  bool armed = false, stopped = true, shutdown = false, arm_refused = false;
  bool mode_changed=false, mode_refused=false;
  bool mode_ready=true;
  bool robot_still=false;
  uint32_t arm_refusal_reason=0;
  double rest_elapsed=0;
  std::array<double,6> measured_velocity{},rest_velocity{};
  uint32_t epoch = 0;
};
struct Feedback {
  Pose pose=identity_pose();
  bool valid=false, still=false, backend_ready=true;
  double rest_elapsed=0;
  std::array<double,6> measured_velocity{},rest_velocity{};
};
class RestMonitor {
 public:
  bool update(double dt,const std::array<double,7>& dq,const std::array<double,6>& measured,
              const std::array<double,6>& commanded,bool use_commanded=true);
  double elapsed() const { return settled_; }
  const std::array<double,6>& filtered_velocity() const { return filtered_; }
 private:
  double settled_=0;
  bool initialized_=false;
  std::array<double,6> filtered_{};
};
class Teleop {
 public:
  Teleop(const Config& config, const Pose& initial);
  void seed_command_pose(const Pose& pose) { target_ = pose; }
  Step update(const Input&, double now, double dt, bool gripper_pause = false,
              bool shutdown = false,const Feedback& feedback=Feedback{});
  const Pose& target() const { return target_; }
  ControlMode mode() const { return mode_; }
 private:
  Config config_;
  Pose target_;
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero(), omega_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d force_ = Eigen::Vector3d::Zero(), moment_ = Eigen::Vector3d::Zero();
  ControlMode mode_;
  bool arm_inhibit_=false, was_paused_=false;
  bool initialized_ = false, armed_ = false, shutdown_ = false;
  uint32_t previous_buttons_ = 0, epoch_ = 0;
};
// Epoch-tagged requests cannot survive a disarm/rearm cycle.
struct GripperChannel {
  std::atomic<uint32_t> authorized_epoch{0};
  std::atomic<uint64_t> request{0};
  std::atomic<uint64_t> completed{0};
  std::atomic<bool> pause{false}, at_rest{true}, fault{false};
  void publish(uint32_t epoch, uint32_t sequence, Grip action);
  bool permits(uint64_t token) const;
  bool pending() const { const auto token=request.load(); return token!=completed.load()&&permits(token); }
};
double steady_time();
}  // namespace gamepad
