#include "franka_gamepad_teleop/force_control.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>

namespace gamepad {
namespace {
Eigen::Vector3d limit(Eigen::Vector3d v,double maximum) {
  if(v.norm()>maximum) v*=maximum/v.norm();
  return v;
}
}
ForceOutput ForceController::update(const Step& step,const ForceState& s,double dt) const {
  ForceOutput out; out.torque=s.previous_torque;
  if(!valid_pose(s.pose)||!valid_pose(step.target)||!std::isfinite(dt)||dt<0||dt>.020) {
    out.valid=false; return out;
  }
  const Eigen::Map<const Eigen::Matrix<double,6,7>> j(s.jacobian.data());
  const Eigen::Map<const Eigen::Matrix<double,7,1>> q(s.q.data()),dq(s.dq.data()),c(s.coriolis.data()),reference(posture_.data());
  const Eigen::Map<const Eigen::Matrix<double,6,1>> bias(step.wrench.data());
  if(!j.allFinite()||!q.allFinite()||!dq.allFinite()||!c.allFinite()||!bias.allFinite()) {
    out.valid=false; return out;
  }
  Eigen::Map<const Eigen::Matrix4d> current(s.pose.data()),target(step.target.data());
  Eigen::Matrix<double,6,1> error;
  error.head<3>()=current.topRightCorner<3,1>()-target.topRightCorner<3,1>();
  const Eigen::Matrix3d r=target.topLeftCorner<3,3>();
  const Eigen::AngleAxisd difference(r.transpose()*current.topLeftCorner<3,3>());
  error.tail<3>()=r*difference.axis()*difference.angle();
  const Eigen::Matrix<double,6,1> velocity=j*dq;
  Eigen::Matrix<double,6,1> wrench;
  wrench.head<3>()=limit(bias.head<3>()-settings_.translation_stiffness*error.head<3>()-
    settings_.translation_damping*velocity.head<3>(),settings_.max_force);
  wrench.tail<3>()=limit(bias.tail<3>()-settings_.rotation_stiffness*error.tail<3>()-
    settings_.rotation_damping*velocity.tail<3>(),settings_.max_torque);
  Eigen::Map<Eigen::Matrix<double,6,1>>(out.task_wrench.data())=wrench;
  Eigen::Matrix<double,6,6> gram=j*j.transpose();
  gram.diagonal().array()+=1e-4;
  const Eigen::Matrix<double,7,7> nullspace=Eigen::Matrix<double,7,7>::Identity()-j.transpose()*gram.ldlt().solve(j);
  Eigen::Matrix<double,7,1> posture=settings_.nullspace_stiffness*(reference-q)-settings_.nullspace_damping*dq;
  for(int i=0;i<7;++i) posture[i]=std::clamp(posture[i],-1.,1.);
  Eigen::Matrix<double,7,1> torque=j.transpose()*wrench+nullspace*posture+c;
  for(size_t i=0;i<7;++i) {
    const double low=settings_.lower[i]+settings_.joint_margin;
    const double high=settings_.upper[i]-settings_.joint_margin;
    if(q[i]<low) torque[i]+=settings_.joint_barrier_stiffness*(low-q[i]);
    if(q[i]>high) torque[i]-=settings_.joint_barrier_stiffness*(q[i]-high);
    if(!std::isfinite(torque[i])||!std::isfinite(s.previous_torque[i])) { out.valid=false;return out; }
    const double bounded=std::clamp(torque[i],-settings_.torque_limits[i],settings_.torque_limits[i]);
    // Preserve exact first-cycle torque, then enforce continuity against the
    // previous torque actually reported by the SDK, including its filtering.
    out.torque[i]=s.previous_torque[i]+std::clamp(bounded-s.previous_torque[i],
      -settings_.torque_rate*dt,settings_.torque_rate*dt);
  }
  return out;
}
}  // namespace gamepad
