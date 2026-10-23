#include "franka_gamepad_teleop/logging.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace gamepad {
static_assert(std::atomic<uint64_t>::is_always_lock_free,"Logging requires lock-free indices");
bool DiagnosticQueue::push(const DiagnosticSample& sample) noexcept {
  const auto w=write_.load(std::memory_order_relaxed);
  if(w-read_.load(std::memory_order_acquire)>=capacity) {
    dropped_.fetch_add(1,std::memory_order_relaxed); return false;
  }
  samples_[w%capacity]=sample;
  write_.store(w+1,std::memory_order_release); return true;
}
bool DiagnosticQueue::pop(DiagnosticSample& sample) noexcept {
  const auto r=read_.load(std::memory_order_relaxed);
  if(r==write_.load(std::memory_order_acquire)) return false;
  sample=samples_[r%capacity];
  read_.store(r+1,std::memory_order_release); return true;
}
namespace {
template<size_t N> void values(std::ostream& stream,const std::array<double,N>& array) {
  for(double value:array) stream<<','<<value;
}
void headers(std::ostream& stream,const std::string& name,size_t size) {
  for(size_t i=0;i<size;++i) stream<<','<<name<<'_'<<i;
}
void header(std::ostream& stream) {
  stream<<"cycle,monotonic_s,robot_time_s,period_s,input_stamp_s,input_connected,buttons,armed,epoch,stopped,shutdown,gripper_intent,has_robot,robot_mode,command_success_rate,dropped_samples";
  headers(stream,"raw_axis",6); headers(stream,"mapped",7);
  for(const auto* name:{"target","measured","commanded","desired"}) headers(stream,name,16);
  for(const auto* name:{"q","q_d","dq","tau_external"}) headers(stream,name,7);
  for(const auto* name:{"commanded_twist","commanded_acceleration","external_wrench"}) headers(stream,name,6);
  stream<<",control_mode,control_backend,mode_ready,mode_changed,mode_refused";
  headers(stream,"requested_velocity",6); headers(stream,"requested_wrench",6);
  headers(stream,"task_wrench",6); headers(stream,"sent_torque",7);
  stream<<'\n';
}
void row(std::ostream& stream,const DiagnosticSample& s,uint64_t dropped) {
  stream<<s.cycle<<','<<s.monotonic<<','<<s.robot_time<<','<<s.period<<','<<s.input.stamp
    <<','<<s.input.connected<<','<<s.input.buttons<<','<<s.step.armed<<','<<s.step.epoch
    <<','<<s.step.stopped<<','<<s.step.shutdown<<','<<static_cast<int>(s.step.grip)
    <<','<<s.has_robot<<','<<s.robot_mode<<','<<s.command_success_rate<<','<<dropped;
  values(stream,s.input.axes); values(stream,s.mapped); values(stream,s.step.target);
  values(stream,s.measured); values(stream,s.commanded); values(stream,s.desired);
  values(stream,s.q); values(stream,s.q_d); values(stream,s.dq); values(stream,s.tau_external);
  values(stream,s.commanded_twist); values(stream,s.commanded_acceleration); values(stream,s.external_wrench);
  stream<<','<<static_cast<int>(s.step.mode)<<','<<static_cast<int>(s.backend)<<','<<s.step.mode_ready
    <<','<<s.step.mode_changed<<','<<s.step.mode_refused;
  values(stream,s.step.velocity); values(stream,s.step.wrench); values(stream,s.task_wrench); values(stream,s.sent_torque);
  stream<<'\n';
}
}
DiagnosticLog::DiagnosticLog(const std::string& path,const std::string& metadata)
  :queue_(std::make_unique<DiagnosticQueue>()),metadata_path_(path+".meta.yaml"),metadata_(metadata) {
  const std::filesystem::path output(path);
  if(std::filesystem::exists(output)||std::filesystem::exists(metadata_path_))
    throw std::runtime_error("Diagnostic log already exists; choose another filename: "+path);
  if(output.has_parent_path()) std::filesystem::create_directories(output.parent_path());
  file_.exceptions(std::ios::badbit|std::ios::failbit); file_.open(path);
  { std::ofstream meta(metadata_path_); meta.exceptions(std::ios::badbit|std::ios::failbit); meta<<metadata_<<'\n'; }
  file_<<std::setprecision(17); header(file_); file_.flush();
  writer_=std::thread([this] {
    try {
      DiagnosticSample sample;
      while(true) {
        bool consumed=false;
        while(queue_->pop(sample)) {
          row(file_,sample,queue_->dropped()); ++rows_; consumed=true;
        }
        if(stop_.load(std::memory_order_acquire)) {
          // Producer has ended before destruction; drain once more after
          // observing stop, including a final publication racing the last pop.
          while(queue_->pop(sample)) { row(file_,sample,queue_->dropped()); ++rows_; }
          break;
        }
        if(consumed) file_.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      file_.flush();
    } catch(const std::exception& error) {
      failed_.store(true);
      std::cerr<<"Diagnostic logging failed (control continues): "<<error.what()<<std::endl;
    }
  });
}
void DiagnosticLog::record(const DiagnosticSample& s) noexcept {
  if(!failed_.load(std::memory_order_relaxed)) queue_->push(s);
}
DiagnosticLog::~DiagnosticLog() {
  stop_.store(true,std::memory_order_release);
  if(writer_.joinable()) writer_.join();
  try {
    auto meta=YAML::Load(metadata_);
    meta["rows_written"]=rows_; meta["dropped_samples"]=queue_->dropped(); meta["writer_failed"]=failed_.load();
    std::ofstream out(metadata_path_); out.exceptions(std::ios::badbit|std::ios::failbit); out<<meta<<'\n';
    std::cout<<"Diagnostic log complete: "<<rows_<<" rows, "<<queue_->dropped()<<" dropped samples"<<std::endl;
  } catch(const std::exception& error) { std::cerr<<"Diagnostic metadata update failed: "<<error.what()<<std::endl; }
}
}  // namespace gamepad
