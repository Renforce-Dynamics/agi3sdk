#include "agi3sdk/c_api.h"

#include "agi3sdk/aimrt_transport.hpp"
#include "agi3sdk/robot_io.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <new>

struct agi3sdk_handle {
  explicit agi3sdk_handle(agi3sdk::Config config)
      : read_only(config.read_only), io(config) {}
  bool read_only;
  agi3sdk::RobotIO io;
  std::unique_ptr<agi3sdk::AimrtTransport> aimrt;
};

namespace {

template <typename Source, typename Destination>
void Copy(const Source& source, Destination& destination) {
  std::copy(std::begin(source), std::end(source), std::begin(destination));
}

agi3sdk::Config ConvertConfig(const agi3sdk_config* input) {
  agi3sdk::Config config;
  if (!input) return config;
  config.state_timeout = std::chrono::milliseconds(input->state_timeout_ms);
  config.command_timeout = std::chrono::milliseconds(input->command_timeout_ms);
  config.neck_kp = input->neck_kp;
  config.neck_kd = input->neck_kd;
  config.safe_damping_kd = input->safe_damping_kd;
  config.read_only = input->read_only != 0;
  config.neck_mode = static_cast<agi3sdk::NeckMode>(input->neck_mode);
  Copy(input->neck_preset_q, config.neck_preset_q);
  config.neck_preset_max_velocity = input->neck_preset_max_velocity;
  return config;
}

bool CopyTopic(const char* input, std::string& output) {
  if (!input || input[0] != '/') return false;
  output = input;
  return true;
}

}  // namespace

extern "C" {

uint32_t agi3sdk_abi_version(void) { return AGI3SDK_ABI_VERSION; }

agi3sdk_handle* agi3sdk_create(const agi3sdk_config* config) {
  try {
    return new agi3sdk_handle(ConvertConfig(config));
  } catch (...) {
    return nullptr;
  }
}

void agi3sdk_destroy(agi3sdk_handle* handle) { delete handle; }

int agi3sdk_start(agi3sdk_handle* handle) {
  if (!handle || !handle->io.Start()) return 0;
  if (handle->aimrt && !handle->aimrt->Start()) {
    handle->io.Stop();
    return 0;
  }
  return 1;
}

void agi3sdk_stop(agi3sdk_handle* handle) {
  if (!handle) return;
  handle->io.Stop();
  if (handle->aimrt) handle->aimrt->Stop();
}

int agi3sdk_configure_aimrt(agi3sdk_handle* handle,
                            const agi3sdk_aimrt_config* config) {
  if (!handle || !config || !config->config_path ||
      config->config_path[0] == '\0' || config->maximum_sync_skew_ns < 0 ||
      (handle->read_only && config->publish_enabled != 0) ||
      handle->io.running()) {
    return 0;
  }
  try {
    agi3sdk::AimrtTransportConfig converted;
    converted.config_path = config->config_path;
    converted.publish_enabled = config->publish_enabled != 0;
    converted.maximum_sync_skew_ns = config->maximum_sync_skew_ns;
    if (!CopyTopic(config->waist_state_topic, converted.waist_state_topic) ||
        !CopyTopic(config->neck_state_topic, converted.neck_state_topic) ||
        !CopyTopic(config->arm_state_topic, converted.arm_state_topic) ||
        !CopyTopic(config->leg_state_topic, converted.leg_state_topic) ||
        !CopyTopic(config->pelvis_imu_topic, converted.pelvis_imu_topic) ||
        !CopyTopic(config->torso_imu_topic, converted.torso_imu_topic) ||
        !CopyTopic(config->waist_command_topic, converted.waist_command_topic) ||
        !CopyTopic(config->neck_command_topic, converted.neck_command_topic) ||
        !CopyTopic(config->arm_command_topic, converted.arm_command_topic) ||
        !CopyTopic(config->leg_command_topic, converted.leg_command_topic)) {
      return 0;
    }
    auto transport =
        std::make_unique<agi3sdk::AimrtTransport>(handle->io, converted);
    if (converted.publish_enabled &&
        !handle->io.SetCommandWriter(
            [ptr = transport.get()](const agi3sdk::HardwareCommand& command) {
              return ptr->WriteCommand(command);
            })) {
      return 0;
    }
    handle->aimrt = std::move(transport);
    return 1;
  } catch (...) {
    return 0;
  }
}

int agi3sdk_get_aimrt_stats(agi3sdk_handle* handle,
                            agi3sdk_aimrt_stats* output) {
  if (!handle || !handle->aimrt || !output) return 0;
  const auto stats = handle->aimrt->Stats();
  output->state_messages = stats.state_messages;
  output->invalid_state_messages = stats.invalid_state_messages;
  output->synchronized_frames = stats.synchronized_frames;
  output->command_batches = stats.command_batches;
  for (std::size_t index = 0; index < agi3sdk::kAimrtStateStreamCount; ++index) {
    output->stream_messages[index] = stats.stream_messages[index];
    output->stream_invalid_messages[index] =
        stats.stream_invalid_messages[index];
    output->stream_receive_age_ns[index] = stats.stream_receive_age_ns[index];
    output->stream_valid_age_ns[index] = stats.stream_valid_age_ns[index];
    output->stream_source_timestamp_ns[index] =
        stats.stream_source_timestamp_ns[index];
  }
  output->latest_source_skew_ns = stats.latest_source_skew_ns;
  return 1;
}

int agi3sdk_set_command_writer(agi3sdk_handle* handle,
                               agi3sdk_command_writer writer,
                               void* user_data) {
  if (!handle || !writer) return 0;
  return handle->io.SetCommandWriter(
      [writer, user_data](const agi3sdk::HardwareCommand& command) {
        agi3sdk_hardware_command converted{};
        Copy(command.q_des, converted.q_des);
        Copy(command.dq_des, converted.dq_des);
        Copy(command.kp, converted.kp);
        Copy(command.kd, converted.kd);
        Copy(command.tau_ff, converted.tau_ff);
        converted.safe_damping = command.safe_damping ? 1 : 0;
        return writer(&converted, user_data) != 0;
      }) ? 1 : 0;
}

int agi3sdk_wait_next_state(agi3sdk_handle* handle, uint64_t last_sequence,
                            int timeout_ms, agi3sdk_policy_state* output) {
  if (!handle || !output || timeout_ms < 0) return 0;
  const auto state = handle->io.WaitNextState(
      last_sequence, std::chrono::milliseconds(timeout_ms));
  if (!state) return 0;
  output->sequence = state->sequence;
  output->timestamp_ns = state->timestamp_ns;
  output->sync_skew_ns = state->sync_skew_ns;
  output->sync_complete = state->sync_complete ? 1 : 0;
  Copy(state->q, output->q);
  Copy(state->dq, output->dq);
  Copy(state->tau_est, output->tau_est);
  Copy(state->quaternion_wxyz, output->quaternion_wxyz);
  Copy(state->gyro, output->gyro);
  Copy(state->acceleration, output->acceleration);
  output->has_torso_imu = state->has_torso_imu ? 1 : 0;
  Copy(state->torso_quaternion_wxyz, output->torso_quaternion_wxyz);
  Copy(state->torso_gyro, output->torso_gyro);
  Copy(state->torso_acceleration, output->torso_acceleration);
  return 1;
}

int agi3sdk_send_policy_command(agi3sdk_handle* handle,
                                const agi3sdk_policy_command* command,
                                uint64_t state_sequence) {
  if (!handle || !command) return -1;
  agi3sdk::PolicyCommand converted;
  Copy(command->q_des, converted.q_des);
  Copy(command->dq_des, converted.dq_des);
  Copy(command->kp, converted.kp);
  Copy(command->kd, converted.kd);
  Copy(command->tau_ff, converted.tau_ff);
  return static_cast<int>(handle->io.SendPolicyCommand(converted, state_sequence));
}

uint64_t agi3sdk_watchdog_trip_count(agi3sdk_handle* handle) {
  return handle ? handle->io.watchdog_trip_count() : 0;
}

int agi3sdk_publish_hardware_state(agi3sdk_handle* handle,
                                   const agi3sdk_hardware_state* state) {
  if (!handle || !state) return 0;
  agi3sdk::RobotState converted;
  converted.sequence = state->sequence;
  converted.timestamp_ns = state->timestamp_ns;
  converted.sync_skew_ns = state->sync_skew_ns;
  converted.sync_complete = state->sync_complete != 0;
  Copy(state->q, converted.q);
  Copy(state->dq, converted.dq);
  Copy(state->tau_est, converted.tau_est);
  Copy(state->pelvis_quaternion_wxyz, converted.pelvis_quaternion_wxyz);
  Copy(state->pelvis_gyro, converted.pelvis_gyro);
  Copy(state->pelvis_acceleration, converted.pelvis_acceleration);
  converted.has_torso_imu = state->has_torso_imu != 0;
  Copy(state->torso_quaternion_wxyz, converted.torso_quaternion_wxyz);
  Copy(state->torso_gyro, converted.torso_gyro);
  Copy(state->torso_acceleration, converted.torso_acceleration);
  return handle->io.PublishState(converted) ? 1 : 0;
}

int agi3sdk_copy_last_hardware_state(agi3sdk_handle* handle,
                                     agi3sdk_hardware_state* output) {
  if (!handle || !output) return 0;
  const auto state = handle->io.LastHardwareState();
  if (!state) return 0;
  output->sequence = state->sequence;
  output->timestamp_ns = state->timestamp_ns;
  output->sync_skew_ns = state->sync_skew_ns;
  output->sync_complete = state->sync_complete ? 1 : 0;
  Copy(state->q, output->q);
  Copy(state->dq, output->dq);
  Copy(state->tau_est, output->tau_est);
  Copy(state->pelvis_quaternion_wxyz, output->pelvis_quaternion_wxyz);
  Copy(state->pelvis_gyro, output->pelvis_gyro);
  Copy(state->pelvis_acceleration, output->pelvis_acceleration);
  output->has_torso_imu = state->has_torso_imu ? 1 : 0;
  Copy(state->torso_quaternion_wxyz, output->torso_quaternion_wxyz);
  Copy(state->torso_gyro, output->torso_gyro);
  Copy(state->torso_acceleration, output->torso_acceleration);
  return 1;
}

int agi3sdk_copy_last_hardware_command(agi3sdk_handle* handle,
                                       agi3sdk_hardware_command* output) {
  if (!handle || !output) return 0;
  const auto command = handle->io.LastHardwareCommand();
  if (!command) return 0;
  Copy(command->q_des, output->q_des);
  Copy(command->dq_des, output->dq_des);
  Copy(command->kp, output->kp);
  Copy(command->kd, output->kd);
  Copy(command->tau_ff, output->tau_ff);
  output->safe_damping = command->safe_damping ? 1 : 0;
  return 1;
}

}  // extern "C"
