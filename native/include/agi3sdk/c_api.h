#pragma once

#include <stdint.h>

#if defined(_WIN32)
#define AGI3SDK_API __declspec(dllexport)
#else
#define AGI3SDK_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define AGI3SDK_POLICY_DOF 29
#define AGI3SDK_HARDWARE_DOF 31
#define AGI3SDK_STATE_STREAMS 6
#define AGI3SDK_ABI_VERSION 5

typedef struct agi3sdk_handle agi3sdk_handle;
typedef struct agi3sdk_hardware_command agi3sdk_hardware_command;
typedef int (*agi3sdk_command_writer)(const agi3sdk_hardware_command*,
                                     void* user_data);

typedef struct {
  int state_timeout_ms;
  int command_timeout_ms;
  double neck_kp;
  double neck_kd;
  double safe_damping_kd;
  int read_only;
  // 0 = hold measured position (damping only), 1 = ramp to neck_preset_q.
  int neck_mode;
  double neck_preset_q[2];  // head_yaw, head_pitch [rad]; pitch > 0 looks down
  double neck_preset_max_velocity;  // [rad/s]
} agi3sdk_config;

typedef struct {
  const char* config_path;
  int publish_enabled;
  int64_t maximum_sync_skew_ns;
  const char* waist_state_topic;
  const char* neck_state_topic;
  const char* arm_state_topic;
  const char* leg_state_topic;
  const char* pelvis_imu_topic;
  const char* torso_imu_topic;
  const char* waist_command_topic;
  const char* neck_command_topic;
  const char* arm_command_topic;
  const char* leg_command_topic;
} agi3sdk_aimrt_config;

typedef struct {
  uint64_t state_messages;
  uint64_t invalid_state_messages;
  uint64_t synchronized_frames;
  uint64_t command_batches;
  // Order: waist, neck, arms, legs, pelvis_imu, torso_imu.
  uint64_t stream_messages[AGI3SDK_STATE_STREAMS];
  uint64_t stream_invalid_messages[AGI3SDK_STATE_STREAMS];
  int64_t stream_receive_age_ns[AGI3SDK_STATE_STREAMS];
  int64_t stream_valid_age_ns[AGI3SDK_STATE_STREAMS];
  int64_t stream_source_timestamp_ns[AGI3SDK_STATE_STREAMS];
  int64_t latest_source_skew_ns;
} agi3sdk_aimrt_stats;

typedef struct {
  uint64_t sequence;
  int64_t timestamp_ns;
  int64_t sync_skew_ns;
  int sync_complete;
  double q[AGI3SDK_POLICY_DOF];
  double dq[AGI3SDK_POLICY_DOF];
  double tau_est[AGI3SDK_POLICY_DOF];
  double quaternion_wxyz[4];
  double gyro[3];
  double acceleration[3];
  int has_torso_imu;
  double torso_quaternion_wxyz[4];
  double torso_gyro[3];
  double torso_acceleration[3];
} agi3sdk_policy_state;

typedef struct {
  uint64_t sequence;
  int64_t timestamp_ns;
  int64_t sync_skew_ns;
  int sync_complete;
  double q[AGI3SDK_HARDWARE_DOF];
  double dq[AGI3SDK_HARDWARE_DOF];
  double tau_est[AGI3SDK_HARDWARE_DOF];
  double pelvis_quaternion_wxyz[4];
  double pelvis_gyro[3];
  double pelvis_acceleration[3];
  int has_torso_imu;
  double torso_quaternion_wxyz[4];
  double torso_gyro[3];
  double torso_acceleration[3];
} agi3sdk_hardware_state;

typedef struct {
  double q_des[AGI3SDK_POLICY_DOF];
  double dq_des[AGI3SDK_POLICY_DOF];
  double kp[AGI3SDK_POLICY_DOF];
  double kd[AGI3SDK_POLICY_DOF];
  double tau_ff[AGI3SDK_POLICY_DOF];
} agi3sdk_policy_command;

struct agi3sdk_hardware_command {
  double q_des[AGI3SDK_HARDWARE_DOF];
  double dq_des[AGI3SDK_HARDWARE_DOF];
  double kp[AGI3SDK_HARDWARE_DOF];
  double kd[AGI3SDK_HARDWARE_DOF];
  double tau_ff[AGI3SDK_HARDWARE_DOF];
  int safe_damping;
};

AGI3SDK_API agi3sdk_handle* agi3sdk_create(const agi3sdk_config* config);
AGI3SDK_API uint32_t agi3sdk_abi_version(void);
AGI3SDK_API void agi3sdk_destroy(agi3sdk_handle* handle);
AGI3SDK_API int agi3sdk_start(agi3sdk_handle* handle);
AGI3SDK_API void agi3sdk_stop(agi3sdk_handle* handle);
// Configure before agi3sdk_start(). A publish-capable transport is rejected
// when the SDK itself was created read-only.
AGI3SDK_API int agi3sdk_configure_aimrt(
    agi3sdk_handle* handle, const agi3sdk_aimrt_config* config);
AGI3SDK_API int agi3sdk_get_aimrt_stats(
    agi3sdk_handle* handle, agi3sdk_aimrt_stats* output);
AGI3SDK_API int agi3sdk_set_command_writer(agi3sdk_handle* handle,
                                           agi3sdk_command_writer writer,
                                           void* user_data);
AGI3SDK_API int agi3sdk_wait_next_state(agi3sdk_handle* handle,
                                        uint64_t last_sequence,
                                        int timeout_ms,
                                        agi3sdk_policy_state* output);
AGI3SDK_API int agi3sdk_send_policy_command(
    agi3sdk_handle* handle, const agi3sdk_policy_command* command,
    uint64_t state_sequence);
AGI3SDK_API uint64_t agi3sdk_watchdog_trip_count(agi3sdk_handle* handle);

// Transport adapter hooks. The AimRT backend calls publish_hardware_state;
// copy_last_hardware_command exists for adapter tests and diagnostics.
AGI3SDK_API int agi3sdk_publish_hardware_state(
    agi3sdk_handle* handle, const agi3sdk_hardware_state* state);
AGI3SDK_API int agi3sdk_copy_last_hardware_command(
    agi3sdk_handle* handle, agi3sdk_hardware_command* output);

#ifdef __cplusplus
}
#endif
