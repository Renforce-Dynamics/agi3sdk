#pragma once

#include "agi3sdk/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace agi3sdk {

template <std::size_t N>
struct JointGroupSample {
  std::int64_t timestamp_ns = 0;
  std::array<double, N> q{};
  std::array<double, N> dq{};
  std::array<double, N> tau_est{};
};

struct ImuSample {
  std::int64_t timestamp_ns = 0;
  std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> gyro{};
  std::array<double, 3> acceleration{0.0, 0.0, 9.81};
};

// Vendor callbacks only convert their message into one of these sample types.
// This dependency-free assembler owns the six-stream synchronization contract.
class StateAssembler {
 public:
  explicit StateAssembler(std::int64_t maximum_skew_ns = 5'000'000);

  void UpdateWaist(const JointGroupSample<3>& sample);
  void UpdateNeck(const JointGroupSample<2>& sample);
  void UpdateArms(const JointGroupSample<14>& sample);
  void UpdateLegs(const JointGroupSample<12>& sample);
  void UpdatePelvisImu(const ImuSample& sample);
  void UpdateTorsoImu(const ImuSample& sample);

  // Returns a frame once for each new aligned input set. An incomplete or
  // over-skew set returns nullopt and never leaks a partial RobotState.
  std::optional<RobotState> TryAssemble();

 private:
  template <typename Sample>
  void Update(std::optional<Sample>& destination, std::uint64_t& generation,
              const Sample& sample);

  std::int64_t maximum_skew_ns_;
  std::uint64_t sequence_ = 0;
  mutable std::mutex mutex_;
  std::optional<JointGroupSample<3>> waist_;
  std::optional<JointGroupSample<2>> neck_;
  std::optional<JointGroupSample<14>> arms_;
  std::optional<JointGroupSample<12>> legs_;
  std::optional<ImuSample> pelvis_imu_;
  std::optional<ImuSample> torso_imu_;
  std::array<std::uint64_t, 6> generations_{};
  std::array<std::uint64_t, 6> emitted_generations_{};
};

}  // namespace agi3sdk
