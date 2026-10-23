#pragma once
#include "franka_gamepad_teleop/core.hpp"
#include <Eigen/Core>

namespace gamepad {
struct ForceState {
  Pose pose=identity_pose();
  std::array<double,42> jacobian{};  // Column-major 6x7, EE Jacobian in base.
  std::array<double,7> q{},dq{},coriolis{},previous_torque{};
};
struct ForceOutput {
  std::array<double,7> torque{};
  std::array<double,6> task_wrench{};
  bool valid=true;
};
// Directional wrench bias with a following hold reference, Cartesian damping,
// orthogonal pose holding, damped nullspace posture hold,
// FR3 joint-limit repulsion and bounded/rate-limited joint torques.
class ForceController {
 public:
  explicit ForceController(const ForceSettings& settings):settings_(settings) {}
  void seed(const ForceState& s) { posture_=s.q; }
  ForceOutput update(const Step&,const ForceState&,double dt) const;
 private:
  ForceSettings settings_;
  std::array<double,7> posture_{};
};
}  // namespace gamepad
