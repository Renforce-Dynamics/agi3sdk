#include "agi3sdk/state_assembler.hpp"

#include <algorithm>
#include <stdexcept>

namespace agi3sdk {

StateAssembler::StateAssembler(std::int64_t maximum_skew_ns)
    : maximum_skew_ns_(maximum_skew_ns) {
  if (maximum_skew_ns_ < 0) {
    throw std::invalid_argument("maximum_skew_ns must be non-negative");
  }
}

template <typename Sample>
void StateAssembler::Update(std::optional<Sample>& destination,
                            std::uint64_t& generation,
                            const Sample& sample) {
  if (sample.timestamp_ns <= 0) return;
  std::lock_guard lock(mutex_);
  if (!destination || sample.timestamp_ns > destination->timestamp_ns) {
    destination = sample;
    ++generation;
  }
}

void StateAssembler::UpdateWaist(const JointGroupSample<3>& sample) {
  Update(waist_, generations_[0], sample);
}
void StateAssembler::UpdateNeck(const JointGroupSample<2>& sample) {
  Update(neck_, generations_[1], sample);
}
void StateAssembler::UpdateArms(const JointGroupSample<14>& sample) {
  Update(arms_, generations_[2], sample);
}
void StateAssembler::UpdateLegs(const JointGroupSample<12>& sample) {
  Update(legs_, generations_[3], sample);
}
void StateAssembler::UpdatePelvisImu(const ImuSample& sample) {
  Update(pelvis_imu_, generations_[4], sample);
}
void StateAssembler::UpdateTorsoImu(const ImuSample& sample) {
  Update(torso_imu_, generations_[5], sample);
}

std::optional<RobotState> StateAssembler::TryAssemble() {
  std::lock_guard lock(mutex_);
  if (!waist_ || !neck_ || !arms_ || !legs_ || !pelvis_imu_ || !torso_imu_) {
    return std::nullopt;
  }
  for (std::size_t index = 0; index < generations_.size(); ++index) {
    if (generations_[index] <= emitted_generations_[index]) {
      return std::nullopt;
    }
  }
  const std::array<std::int64_t, 6> timestamps = {
      waist_->timestamp_ns, neck_->timestamp_ns, arms_->timestamp_ns,
      legs_->timestamp_ns, pelvis_imu_->timestamp_ns, torso_imu_->timestamp_ns,
  };
  const auto [oldest, latest] =
      std::minmax_element(timestamps.begin(), timestamps.end());
  if (*latest - *oldest > maximum_skew_ns_) {
    return std::nullopt;
  }

  RobotState state;
  state.sequence = ++sequence_;
  state.timestamp_ns = *latest;
  state.sync_skew_ns = *latest - *oldest;
  state.sync_complete = true;

  auto copy_group = [&state](const auto& sample, std::size_t offset) {
    for (std::size_t index = 0; index < sample.q.size(); ++index) {
      state.q[offset + index] = sample.q[index];
      state.dq[offset + index] = sample.dq[index];
      state.tau_est[offset + index] = sample.tau_est[index];
    }
  };
  copy_group(*waist_, 0);
  copy_group(*neck_, 3);
  copy_group(*arms_, 5);
  copy_group(*legs_, 19);
  state.pelvis_quaternion_wxyz = pelvis_imu_->quaternion_wxyz;
  state.pelvis_gyro = pelvis_imu_->gyro;
  state.pelvis_acceleration = pelvis_imu_->acceleration;
  state.has_torso_imu = true;
  state.torso_quaternion_wxyz = torso_imu_->quaternion_wxyz;
  state.torso_gyro = torso_imu_->gyro;
  state.torso_acceleration = torso_imu_->acceleration;
  emitted_generations_ = generations_;
  return state;
}

}  // namespace agi3sdk
