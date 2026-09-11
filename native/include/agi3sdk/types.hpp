#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace agi3sdk {

inline constexpr std::size_t kPolicyDof = 29;
inline constexpr std::size_t kHardwareDof = 31;

// Runtime policy order: legs(12), waist(3), arms(14).
// A3 body-drive order: waist(3), neck(2), arms(14), legs(12).
inline constexpr std::array<std::size_t, kPolicyDof> kPolicyToHardware = {
    19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    0, 1, 2,
    5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
};

inline constexpr std::array<const char*, 3> kWaistJointNames = {
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"};
inline constexpr std::array<const char*, 2> kNeckJointNames = {
    "head_yaw_joint", "head_pitch_joint"};
inline constexpr std::array<const char*, 14> kArmJointNames = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
inline constexpr std::array<const char*, 12> kLegJointNames = {
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};

struct RobotState {
  std::uint64_t sequence = 0;
  std::int64_t timestamp_ns = 0;
  std::int64_t sync_skew_ns = 0;
  bool sync_complete = false;
  std::array<double, kHardwareDof> q{};
  std::array<double, kHardwareDof> dq{};
  std::array<double, kHardwareDof> tau_est{};
  std::array<double, 4> pelvis_quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> pelvis_gyro{};
  std::array<double, 3> pelvis_acceleration{0.0, 0.0, 9.81};
  bool has_torso_imu = false;
  std::array<double, 4> torso_quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> torso_gyro{};
  std::array<double, 3> torso_acceleration{0.0, 0.0, 9.81};
};

struct PolicyState {
  std::uint64_t sequence = 0;
  std::int64_t timestamp_ns = 0;
  std::int64_t sync_skew_ns = 0;
  bool sync_complete = false;
  std::array<double, kPolicyDof> q{};
  std::array<double, kPolicyDof> dq{};
  std::array<double, kPolicyDof> tau_est{};
  // Pelvis IMU is kept as the legacy policy orientation input.
  std::array<double, 4> quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> gyro{};
  std::array<double, 3> acceleration{0.0, 0.0, 9.81};
  // Torso IMU is exposed separately for upper-body/waist stabilization layers.
  bool has_torso_imu = false;
  std::array<double, 4> torso_quaternion_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> torso_gyro{};
  std::array<double, 3> torso_acceleration{0.0, 0.0, 9.81};
};

struct PolicyCommand {
  std::array<double, kPolicyDof> q_des{};
  std::array<double, kPolicyDof> dq_des{};
  std::array<double, kPolicyDof> kp{};
  std::array<double, kPolicyDof> kd{};
  std::array<double, kPolicyDof> tau_ff{};
};

struct HardwareCommand {
  std::array<double, kHardwareDof> q_des{};
  std::array<double, kHardwareDof> dq_des{};
  std::array<double, kHardwareDof> kp{};
  std::array<double, kHardwareDof> kd{};
  std::array<double, kHardwareDof> tau_ff{};
  bool safe_damping = false;
};

}  // namespace agi3sdk
