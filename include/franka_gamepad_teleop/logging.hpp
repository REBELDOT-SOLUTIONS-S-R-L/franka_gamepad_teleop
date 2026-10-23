#pragma once
#include "franka_gamepad_teleop/core.hpp"
#include <array>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

namespace gamepad {
struct DiagnosticSample {
  uint64_t cycle=0;
  double monotonic=0,robot_time=0,period=0;
  Input input;
  std::array<double,7> mapped{};
  Step step;
  bool has_robot=false;
  int robot_mode=0;
  Pose measured{},commanded{},desired{};
  std::array<double,7> q{},q_d{},dq{},tau_external{};
  std::array<double,6> commanded_twist{},commanded_acceleration{},external_wrench{};
  double command_success_rate=0;
};
// One producer (control callback), one consumer (file writer). The fixed queue
// drops samples when full; the producer never waits, allocates, or writes files.
class DiagnosticQueue {
 public:
  static constexpr size_t capacity=2048;
  bool push(const DiagnosticSample&) noexcept;
  bool pop(DiagnosticSample&) noexcept;
  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
 private:
  std::array<DiagnosticSample,capacity> samples_{};
  alignas(64) std::atomic<uint64_t> write_{0};
  alignas(64) std::atomic<uint64_t> read_{0};
  std::atomic<uint64_t> dropped_{0};
};
class DiagnosticLog {
 public:
  DiagnosticLog(const std::string& path,const std::string& metadata);
  ~DiagnosticLog();
  void record(const DiagnosticSample& s) noexcept;
 private:
  std::unique_ptr<DiagnosticQueue> queue_;
  std::ofstream file_;
  std::string metadata_path_,metadata_;
  std::atomic<bool> stop_{false},failed_{false};
  uint64_t rows_=0;
  std::thread writer_;
};
}  // namespace gamepad
