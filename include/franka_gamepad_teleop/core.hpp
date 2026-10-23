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
struct Config {
  std::string name;
  double stick_deadzone = .12, trigger_deadzone = .04, poll_hz = 200, timeout = .10;
  double speed = .020, rotation = 20*kPi/180, acceleration = .05;
  double angular_acceleration = 60*kPi/180, cutoff = 10;
  std::array<std::vector<Binding>, 6> bindings;
  int arm = 3, close = 0, open = 1;  // Standard SDL A, B, X, Y indices.
  bool gripper = true;
  double force = 30, gripper_speed = .04, epsilon_inner = .001, epsilon_outer = .08;
};
Config load_config(const std::string& path);
int source_index(const std::string& name);
int button_index(const std::string& name);
double deadzone(double value, double zone, bool trigger);
std::array<double, 6> map_input(const Config&, const Input&);
Pose identity_pose();
bool valid_pose(const Pose&);
enum class Grip : uint8_t { None = 0, Close = 1, Open = 2 };
struct Step {
  Pose target;
  Grip grip = Grip::None;
  bool armed = false, stopped = true, shutdown = false, arm_refused = false;
  uint32_t epoch = 0;
};
class Delta {
 public:
  Delta(const Config& config, const Pose& initial);
  void seed_command_pose(const Pose& pose) { target_ = pose; }
  Step update(const Input&, double now, double dt, bool gripper_pause = false,
              bool shutdown = false);
  const Pose& target() const { return target_; }
 private:
  Config config_;
  Pose target_;
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero(), omega_ = Eigen::Vector3d::Zero();
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
