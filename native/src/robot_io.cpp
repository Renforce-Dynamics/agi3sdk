#include "agi3sdk/robot_io.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace agi3sdk {
namespace {

template <std::size_t N>
bool Finite(const std::array<double, N>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

}  // namespace

RobotIO::RobotIO(Config config, CommandWriter writer)
    : config_(config), writer_(std::move(writer)) {}

RobotIO::~RobotIO() { Stop(); }

bool RobotIO::Start() {
  std::lock_guard lock(mutex_);
  if (running_) return true;
  if (config_.state_timeout.count() <= 0 ||
      config_.command_timeout.count() <= 0 ||
      !std::isfinite(config_.neck_kp) || config_.neck_kp < 0.0 ||
      !std::isfinite(config_.neck_kd) || config_.neck_kd < 0.0 ||
      !std::isfinite(config_.safe_damping_kd) ||
      config_.safe_damping_kd < 0.0) {
    return false;
  }
  running_ = true;
  watchdog_thread_ = std::thread(&RobotIO::WatchdogLoop, this);
  return true;
}

void RobotIO::Stop() {
  std::optional<HardwareCommand> final_damping;
  {
    std::lock_guard lock(mutex_);
    if (!running_) return;
    if (!config_.read_only && have_state_ && writer_) {
      final_damping = MakeSafeDamping(state_);
    }
    running_ = false;
  }
  state_cv_.notify_all();
  watchdog_cv_.notify_all();
  if (watchdog_thread_.joinable()) watchdog_thread_.join();
  if (final_damping) WriteCommand(*final_damping);
}

bool RobotIO::running() const {
  std::lock_guard lock(mutex_);
  return running_;
}

bool RobotIO::SetCommandWriter(CommandWriter writer) {
  std::lock_guard lock(mutex_);
  if (running_) return false;
  writer_ = std::move(writer);
  return true;
}

bool RobotIO::PublishState(const RobotState& state) {
  if (state.sequence == 0 || !IsFinite(state)) return false;
  {
    std::lock_guard lock(mutex_);
    if (!running_ || (have_state_ && state.sequence <= state_.sequence)) {
      return false;
    }
    state_ = state;
    have_state_ = true;
    state_received_at_ = Clock::now();
  }
  state_cv_.notify_all();
  watchdog_cv_.notify_all();
  return true;
}

std::optional<PolicyState> RobotIO::WaitNextState(
    std::uint64_t last_sequence, std::chrono::milliseconds timeout) {
  std::unique_lock lock(mutex_);
  state_cv_.wait_for(lock, timeout, [&] {
    return !running_ || (have_state_ && state_.sequence > last_sequence);
  });
  if (!running_ || !have_state_ || state_.sequence <= last_sequence) {
    return std::nullopt;
  }
  delivered_state_ = state_;
  delivered_state_received_at_ = state_received_at_;
  have_delivered_state_ = true;
  return ExtractPolicyState(delivered_state_);
}

std::optional<PolicyState> RobotIO::LatestState() const {
  std::lock_guard lock(mutex_);
  if (!have_state_) return std::nullopt;
  return ExtractPolicyState(state_);
}

SendResult RobotIO::SendPolicyCommand(const PolicyCommand& command,
                                      std::uint64_t state_sequence) {
  HardwareCommand expanded;
  {
    std::lock_guard lock(mutex_);
    if (!running_) return SendResult::kNotRunning;
    if (config_.read_only) return SendResult::kNotRunning;
    if (!have_state_) return SendResult::kNoState;
    // AimRT state callbacks run much faster than the policy loop. Validate
    // against the immutable snapshot actually returned to the policy, not the
    // newest callback that may arrive while Python is doing inference.
    if (!have_delivered_state_ ||
        state_sequence != delivered_state_.sequence) {
      return SendResult::kStateSequenceMismatch;
    }
    if (Clock::now() - delivered_state_received_at_ > config_.state_timeout) {
      return SendResult::kStateStale;
    }
    if (!IsFinite(command)) return SendResult::kInvalidCommand;
    expanded = ExpandPolicyCommand(command, delivered_state_);
  }
  if (!WriteCommand(expanded)) return SendResult::kTransportError;
  {
    std::lock_guard lock(mutex_);
    command_sent_at_ = Clock::now();
    have_command_ = true;
    watchdog_latched_ = false;
  }
  watchdog_cv_.notify_all();
  return SendResult::kOk;
}

std::optional<HardwareCommand> RobotIO::LastHardwareCommand() const {
  std::lock_guard lock(mutex_);
  return last_command_;
}

std::uint64_t RobotIO::watchdog_trip_count() const {
  std::lock_guard lock(mutex_);
  return watchdog_trip_count_;
}

bool RobotIO::IsFinite(const RobotState& state) {
  return Finite(state.q) && Finite(state.dq) && Finite(state.tau_est) &&
         Finite(state.pelvis_quaternion_wxyz) && Finite(state.pelvis_gyro) &&
         Finite(state.pelvis_acceleration) &&
         (!state.has_torso_imu ||
          (Finite(state.torso_quaternion_wxyz) && Finite(state.torso_gyro) &&
           Finite(state.torso_acceleration)));
}

bool RobotIO::IsFinite(const PolicyCommand& command) {
  return Finite(command.q_des) && Finite(command.dq_des) &&
         Finite(command.kp) && Finite(command.kd) && Finite(command.tau_ff) &&
         std::all_of(command.kp.begin(), command.kp.end(),
                     [](double value) { return value >= 0.0; }) &&
         std::all_of(command.kd.begin(), command.kd.end(),
                     [](double value) { return value >= 0.0; });
}

PolicyState RobotIO::ExtractPolicyState(const RobotState& state) {
  PolicyState result;
  result.sequence = state.sequence;
  result.timestamp_ns = state.timestamp_ns;
  result.sync_skew_ns = state.sync_skew_ns;
  result.sync_complete = state.sync_complete;
  result.quaternion_wxyz = state.pelvis_quaternion_wxyz;
  result.gyro = state.pelvis_gyro;
  result.acceleration = state.pelvis_acceleration;
  result.has_torso_imu = state.has_torso_imu;
  result.torso_quaternion_wxyz = state.torso_quaternion_wxyz;
  result.torso_gyro = state.torso_gyro;
  result.torso_acceleration = state.torso_acceleration;
  for (std::size_t policy = 0; policy < kPolicyDof; ++policy) {
    const auto hardware = kPolicyToHardware[policy];
    result.q[policy] = state.q[hardware];
    result.dq[policy] = state.dq[hardware];
    result.tau_est[policy] = state.tau_est[hardware];
  }
  return result;
}

HardwareCommand RobotIO::ExpandPolicyCommand(const PolicyCommand& command,
                                             const RobotState& state) const {
  HardwareCommand result;
  for (std::size_t policy = 0; policy < kPolicyDof; ++policy) {
    const auto hardware = kPolicyToHardware[policy];
    result.q_des[hardware] = command.q_des[policy];
    result.dq_des[hardware] = command.dq_des[policy];
    result.kp[hardware] = command.kp[policy];
    result.kd[hardware] = command.kd[policy];
    result.tau_ff[hardware] = command.tau_ff[policy];
  }
  for (std::size_t neck = 3; neck <= 4; ++neck) {
    result.q_des[neck] = state.q[neck];
    result.kp[neck] = config_.neck_kp;
    result.kd[neck] = config_.neck_kd;
  }
  return result;
}

HardwareCommand RobotIO::MakeSafeDamping(const RobotState& state) const {
  HardwareCommand result;
  result.q_des = state.q;
  result.kd.fill(config_.safe_damping_kd);
  result.safe_damping = true;
  return result;
}

bool RobotIO::WriteCommand(const HardwareCommand& command) {
  if (!writer_ || !writer_(command)) return false;
  std::lock_guard lock(mutex_);
  last_command_ = command;
  return true;
}

void RobotIO::WatchdogLoop() {
  std::unique_lock lock(mutex_);
  while (running_) {
    watchdog_cv_.wait_for(lock, config_.command_timeout / 4);
    if (!running_ || config_.read_only || !have_state_ || !have_command_ ||
        watchdog_latched_) {
      continue;
    }
    if (Clock::now() - command_sent_at_ <= config_.command_timeout) continue;
    const auto safe = MakeSafeDamping(state_);
    watchdog_latched_ = true;
    ++watchdog_trip_count_;
    lock.unlock();
    WriteCommand(safe);
    lock.lock();
  }
}

const char* ToString(SendResult result) noexcept {
  switch (result) {
    case SendResult::kOk: return "ok";
    case SendResult::kNotRunning: return "not running or read-only";
    case SendResult::kNoState: return "no state";
    case SendResult::kStateSequenceMismatch: return "state sequence mismatch";
    case SendResult::kStateStale: return "state stale";
    case SendResult::kInvalidCommand: return "invalid command";
    case SendResult::kTransportError: return "transport error";
  }
  return "unknown";
}

}  // namespace agi3sdk
