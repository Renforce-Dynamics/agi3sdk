#pragma once

#include "agi3sdk/types.hpp"

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace agi3sdk {

enum class SendResult {
  kOk = 0,
  kNotRunning,
  kNoState,
  kStateSequenceMismatch,
  kStateStale,
  kInvalidCommand,
  kTransportError,
};

struct Config {
  std::chrono::milliseconds state_timeout{50};
  std::chrono::milliseconds command_timeout{100};
  double neck_kp = 20.0;
  double neck_kd = 1.0;
  double safe_damping_kd = 3.0;
  bool read_only = true;
};

class RobotIO {
 public:
  using CommandWriter = std::function<bool(const HardwareCommand&)>;

  explicit RobotIO(Config config = {}, CommandWriter writer = {});
  ~RobotIO();

  RobotIO(const RobotIO&) = delete;
  RobotIO& operator=(const RobotIO&) = delete;

  bool Start();
  void Stop();
  bool running() const;
  bool SetCommandWriter(CommandWriter writer);

  // Called by the hardware transport only after all six A3 state streams have
  // been synchronized into one immutable 31-DOF snapshot.
  bool PublishState(const RobotState& state);
  std::optional<PolicyState> WaitNextState(
      std::uint64_t last_sequence, std::chrono::milliseconds timeout);
  std::optional<PolicyState> LatestState() const;

  SendResult SendPolicyCommand(const PolicyCommand& command,
                               std::uint64_t state_sequence);
  std::optional<HardwareCommand> LastHardwareCommand() const;
  std::uint64_t watchdog_trip_count() const;

 private:
  using Clock = std::chrono::steady_clock;

  static bool IsFinite(const RobotState& state);
  static bool IsFinite(const PolicyCommand& command);
  static PolicyState ExtractPolicyState(const RobotState& state);
  HardwareCommand ExpandPolicyCommand(const PolicyCommand& command,
                                      const RobotState& state) const;
  HardwareCommand MakeSafeDamping(const RobotState& state) const;
  bool WriteCommand(const HardwareCommand& command);
  void WatchdogLoop();

  Config config_;
  CommandWriter writer_;
  mutable std::mutex mutex_;
  std::condition_variable state_cv_;
  std::condition_variable watchdog_cv_;
  bool running_ = false;
  bool have_state_ = false;
  bool have_command_ = false;
  bool watchdog_latched_ = false;
  RobotState state_{};
  Clock::time_point state_received_at_{};
  bool have_delivered_state_ = false;
  RobotState delivered_state_{};
  Clock::time_point delivered_state_received_at_{};
  Clock::time_point command_sent_at_{};
  std::optional<HardwareCommand> last_command_;
  std::uint64_t watchdog_trip_count_ = 0;
  std::thread watchdog_thread_;
};

const char* ToString(SendResult result) noexcept;

}  // namespace agi3sdk
