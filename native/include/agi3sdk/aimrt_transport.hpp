#pragma once

#include "agi3sdk/types.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace agi3sdk {

class RobotIO;

inline constexpr std::size_t kAimrtStateStreamCount = 6;

struct AimrtTransportConfig {
  std::string config_path;
  bool publish_enabled = false;
  std::int64_t maximum_sync_skew_ns = 5'000'000;
  std::string waist_state_topic;
  std::string neck_state_topic;
  std::string arm_state_topic;
  std::string leg_state_topic;
  std::string pelvis_imu_topic;
  std::string torso_imu_topic;
  std::string waist_command_topic;
  std::string neck_command_topic;
  std::string arm_command_topic;
  std::string leg_command_topic;
};

struct AimrtTransportStats {
  std::uint64_t state_messages = 0;
  std::uint64_t invalid_state_messages = 0;
  std::uint64_t synchronized_frames = 0;
  std::uint64_t command_batches = 0;
  // Order: waist, neck, arms, legs, pelvis_imu, torso_imu.
  std::array<std::uint64_t, kAimrtStateStreamCount> stream_messages{};
  std::array<std::uint64_t, kAimrtStateStreamCount> stream_invalid_messages{};
  std::array<std::int64_t, kAimrtStateStreamCount> stream_receive_age_ns{};
  std::array<std::int64_t, kAimrtStateStreamCount> stream_valid_age_ns{};
  std::array<std::int64_t, kAimrtStateStreamCount> stream_source_timestamp_ns{};
  std::int64_t latest_source_skew_ns = -1;
};

// Owns the target-specific AimRT Core and the six state/four command topics.
// AimRT and vendor message headers stay behind this PIMPL boundary; Python
// only sees one fixed-size PolicyState/PolicyCommand per control period.
class AimrtTransport {
 public:
  AimrtTransport(RobotIO& io, AimrtTransportConfig config);
  ~AimrtTransport();

  AimrtTransport(const AimrtTransport&) = delete;
  AimrtTransport& operator=(const AimrtTransport&) = delete;

  bool Start();
  void Stop();
  bool WriteCommand(const HardwareCommand& command);
  AimrtTransportStats Stats() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace agi3sdk
