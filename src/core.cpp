#include "franka_gamepad_teleop/core.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <set>

namespace gamepad {
namespace {
const std::array<std::string, 6> axes = {"left_x", "left_y", "right_x", "right_y", "left_trigger", "right_trigger"};
const std::array<std::string, 15> buttons = {"a", "b", "x", "y", "back", "guide", "start", "left_stick", "right_stick", "left_shoulder", "right_shoulder", "dpad_up", "dpad_down", "dpad_left", "dpad_right"};
double positive(const YAML::Node& n, const char* key) {
  const double x=n[key].as<double>();
  if(!std::isfinite(x) || x<=0) throw std::runtime_error(std::string(key)+" must be finite and positive");
  return x;
}
void keys(const YAML::Node& node, std::initializer_list<const char*> allowed) {
  if(!node.IsMap()) throw std::runtime_error("Expected a YAML mapping");
  for(const auto& item:node) {
    const auto key=item.first.as<std::string>();
    if(std::none_of(allowed.begin(),allowed.end(),[&](const char* s){return key==s;}))
      throw std::runtime_error("Unknown YAML setting: "+key);
  }
}
Eigen::Vector3d bounded(Eigen::Vector3d v) { if(v.norm()>1) v.normalize(); return v; }
Eigen::Vector3d slew(const Eigen::Vector3d& old, const Eigen::Vector3d& goal, double step) {
  const Eigen::Vector3d d=goal-old;
  if(d.norm()<=step) return goal;
  return old+d*(step/d.norm());
}
}
int button_index(const std::string& name) {
  const auto p=std::find(buttons.begin(),buttons.end(),name);
  if(p==buttons.end()) throw std::runtime_error("Unknown button: "+name);
  return static_cast<int>(p-buttons.begin());
}
int source_index(const std::string& name) {
  const auto p=std::find(axes.begin(),axes.end(),name);
  return p!=axes.end()?static_cast<int>(p-axes.begin()):6+button_index(name);
}
Config load_config(const std::string& path) {
  const auto root=YAML::LoadFile(path);
  keys(root,{"mode","gamepad","motion","bindings","gripper"});
  if(root["mode"].as<std::string>()!="delta") throw std::runtime_error("Only delta control is implemented");
  Config c;
  const auto g=root["gamepad"],m=root["motion"],b=root["bindings"],h=root["gripper"];
  keys(g,{"name_contains","stick_deadzone","trigger_deadzone","poll_hz","stale_timeout_s"});
  c.name=g["name_contains"].as<std::string>();
  c.stick_deadzone=g["stick_deadzone"].as<double>(); c.trigger_deadzone=g["trigger_deadzone"].as<double>();
  for(double z:{c.stick_deadzone,c.trigger_deadzone})
    if(!std::isfinite(z)||z<0||z>=1) throw std::runtime_error("Deadzones must be in [0,1)");
  c.poll_hz=positive(g,"poll_hz"); c.timeout=positive(g,"stale_timeout_s");
  if(c.poll_hz<20||c.poll_hz>1000||c.timeout<3/c.poll_hz||c.timeout>.5)
    throw std::runtime_error("Use 20-1000 Hz polling and a timeout of 3 polling periods to 0.5 s");
  keys(m,{"max_translation_m_s","max_rotation_deg_s","translation_acceleration_m_s2","rotation_acceleration_deg_s2","lowpass_cutoff_hz"});
  c.speed=positive(m,"max_translation_m_s"); c.rotation=positive(m,"max_rotation_deg_s")*kPi/180;
  c.acceleration=positive(m,"translation_acceleration_m_s2");
  c.angular_acceleration=positive(m,"rotation_acceleration_deg_s2")*kPi/180;
  c.cutoff=positive(m,"lowpass_cutoff_hz");
  if(c.cutoff>100) throw std::runtime_error("lowpass_cutoff_hz must be at most 100 Hz");
  if(c.speed>1.7||c.rotation>2.5||c.acceleration>8.1||c.angular_acceleration>12.5)
    throw std::runtime_error("Motion settings exceed this application's Cartesian interface ceilings");
  keys(b,{"translation_x","translation_y","translation_z","rotation_x","rotation_y","rotation_z","translation_tcp_z","arm_toggle","gripper_close","gripper_open"});
  c.arm=button_index(b["arm_toggle"].as<std::string>());
  c.close=button_index(b["gripper_close"].as<std::string>());
  c.open=button_index(b["gripper_open"].as<std::string>());
  if(std::set<int>{c.arm,c.close,c.open}.size()!=3) throw std::runtime_error("Action buttons must be distinct");
  const std::array<const char*,7> commands={"translation_x","translation_y","translation_z","rotation_x","rotation_y","rotation_z","translation_tcp_z"};
  for(size_t i=0;i<commands.size();++i) {
    const auto list=b[commands[i]];
    // Older profiles remain usable without the additional local-Z binding.
    if(i==6&&!list) continue;
    if(!list.IsSequence()||list.size()==0) throw std::runtime_error("Each motion binding needs at least one source");
    for(const auto& n:list) {
      keys(n,{"source","scale"});
      const int source=source_index(n["source"].as<std::string>());
      const double scale=n["scale"].as<double>();
      if(!std::isfinite(scale)||std::abs(scale)>1||scale==0) throw std::runtime_error("Binding scale must be nonzero and in [-1,1]");
      if(source>=6&&(source-6==c.arm||source-6==c.close||source-6==c.open))
        throw std::runtime_error("Action buttons cannot also command motion");
      c.bindings[i].push_back({source,scale});
    }
  }
  keys(h,{"enabled","force_N","speed_m_s","epsilon_inner_m","epsilon_outer_m"});
  c.gripper=h["enabled"].as<bool>(); c.force=positive(h,"force_N");
  c.gripper_speed=positive(h,"speed_m_s"); c.epsilon_inner=positive(h,"epsilon_inner_m");
  c.epsilon_outer=positive(h,"epsilon_outer_m");
  if(c.force<30||c.force>70||c.gripper_speed>.1||c.epsilon_inner>.08||c.epsilon_outer>.08)
    throw std::runtime_error("Invalid Franka Hand force, speed or grasp tolerances");
  return c;
}
double deadzone(double x,double z,bool trigger) {
  if(!std::isfinite(x)) return 0;
  x=std::clamp(x,trigger?0.:-1.,1.);
  return std::abs(x)<=z?0:std::copysign((std::abs(x)-z)/(1-z),x);
}
std::array<double,7> map_input(const Config& c,const Input& s) {
  std::array<double,7> out{};
  for(size_t i=0;i<out.size();++i) {
    for(const auto& b:c.bindings[i]) {
      const double value=b.source<6?deadzone(s.axes[b.source],b.source>=4?c.trigger_deadzone:c.stick_deadzone,b.source>=4)
        :static_cast<double>((s.buttons>>(b.source-6))&1U);
      out[i]+=b.scale*value;
    }
    out[i]=std::clamp(out[i],-1.,1.);
  }
  return out;
}
Pose identity_pose() { Pose p{}; p[0]=p[5]=p[10]=p[15]=1; return p; }
bool valid_pose(const Pose& p) {
  Eigen::Map<const Eigen::Matrix4d> t(p.data());
  const Eigen::Matrix3d r=t.topLeftCorner<3,3>();
  return t.allFinite()&&(r.transpose()*r-Eigen::Matrix3d::Identity()).norm()<1e-6&&
    std::abs(r.determinant()-1)<1e-6&&
    (t.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-9;
}
Delta::Delta(const Config& c,const Pose& p):config_(c),target_(p) {
  if(!valid_pose(p)) throw std::runtime_error("Initial TCP pose is not a finite rigid transform");
}
Step Delta::update(const Input& s,double now,double dt,bool pause,bool exit) {
  Step out{};
  const bool valid=s.connected&&std::isfinite(s.stamp)&&std::isfinite(now)&&
    now-s.stamp>=-.002&&now-s.stamp<=config_.timeout&&
    std::all_of(s.axes.begin(),s.axes.end(),[](double v){return std::isfinite(v)&&v>=-1&&v<=1;});
  shutdown_=shutdown_||exit||!valid||!std::isfinite(dt)||dt<0||dt>.020;
  if(shutdown_) armed_=false;
  const double step=std::isfinite(dt)?std::clamp(dt,0.,.020):.001;
  const auto mapped=valid?map_input(config_,s):std::array<double,7>{};
  const uint32_t rising=initialized_?(s.buttons&~previous_buttons_):0;
  // Opposing triggers/buttons may cancel mathematically while still being held.
  bool neutral=true;
  for(const auto& axis:config_.bindings) for(const auto& binding:axis) {
    const int i=binding.source;
    if(i<6) neutral=neutral&&std::abs(deadzone(s.axes[i],i>=4?config_.trigger_deadzone:config_.stick_deadzone,i>=4))<1e-10;
    else neutral=neutral&&!(s.buttons&(1U<<(i-6)));
  }
  const bool at_rest=velocity_.norm()<1e-10&&omega_.norm()<1e-10;
  if(valid&&!shutdown_&&(rising&(1U<<config_.arm))) {
    if(armed_) armed_=false;
    else if(neutral&&at_rest&&!pause&&!(s.buttons&((1U<<config_.close)|(1U<<config_.open)))) {
      armed_=true; ++epoch_;
    } else out.arm_refused=true;
  }
  if(valid) { previous_buttons_=s.buttons; initialized_=true; }
  if(armed_&&config_.gripper) {
    if(rising&(1U<<config_.open)) out.grip=Grip::Open;
    else if(rising&(1U<<config_.close)) out.grip=Grip::Close;
  }
  const bool moving=armed_&&!pause&&out.grip==Grip::None;
  Eigen::Map<Eigen::Matrix4d> t(target_.data());
  Eigen::Vector3d v=Eigen::Vector3d::Zero(),w=Eigen::Vector3d::Zero();
  if(moving) {
    // Blend base translation with the TCP's current local Z direction, then
    // cap the combined vector so simultaneous inputs retain the speed limit.
    const Eigen::Vector3d combined=Eigen::Vector3d(mapped[0],mapped[1],mapped[2])+
      t.topLeftCorner<3,3>().col(2)*mapped[6];
    v=bounded(combined)*config_.speed;
    w=bounded({mapped[3],mapped[4],mapped[5]})*config_.rotation;
  }
  velocity_=slew(velocity_,v,config_.acceleration*step);
  omega_=slew(omega_,w,config_.angular_acceleration*step);
  t.topRightCorner<3,1>()+=velocity_*step;
  const double angle=omega_.norm()*step;
  if(angle>0) {
    // Post-multiply: each increment rotates about the TCP's local axes.
    const Eigen::Matrix3d next=t.topLeftCorner<3,3>()*Eigen::AngleAxisd(angle,omega_.normalized()).toRotationMatrix();
    t.topLeftCorner<3,3>()=Eigen::Quaterniond(next).normalized().toRotationMatrix();
  }
  out.target=target_; out.armed=armed_; out.epoch=epoch_; out.shutdown=shutdown_;
  out.stopped=velocity_.norm()<1e-10&&omega_.norm()<1e-10;
  return out;
}
void InputBuffer::store(const Input& s) {
  version_.fetch_add(1);
  for(size_t i=0;i<6;++i) axes_[i].store(s.axes[i]);
  buttons_.store(s.buttons); stamp_.store(s.stamp); connected_.store(s.connected);
  version_.fetch_add(1);
}
Input InputBuffer::load() const {
  for(int attempt=0;attempt<3;++attempt) {
    const auto before=version_.load(); if(before&1U) continue;
    Input s;
    for(size_t i=0;i<6;++i) s.axes[i]=axes_[i].load();
    s.buttons=buttons_.load(); s.stamp=stamp_.load(); s.connected=connected_.load();
    if(version_.load()==before) return s;
  }
  return {};  // A busy/stalled producer cannot supply a command.
}
void GripperChannel::publish(uint32_t epoch,uint32_t sequence,Grip action) {
  pause.store(true);
  request.store((static_cast<uint64_t>(epoch)<<32)|(static_cast<uint64_t>(sequence&0x3fffffffU)<<2)|static_cast<uint8_t>(action));
}
bool GripperChannel::permits(uint64_t token) const {
  return !fault.load()&&(token&3U)!=0&&authorized_epoch.load()!=0&&
    static_cast<uint32_t>(token>>32)==authorized_epoch.load()&&request.load()==token;
}
double steady_time() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace gamepad
