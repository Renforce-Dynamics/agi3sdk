#include "agi3sdk/aimrt_transport.hpp"

#include "agi3sdk/robot_io.hpp"
#include "agi3sdk/state_assembler.hpp"

#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string_view>

#ifdef AGI3SDK_WITH_AIMRT
#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "core/aimrt_core.h"
#include "joint_msgs/msg/detail/joint_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "joint_msgs/msg/detail/joint_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "joint_msgs/msg/joint_command.hpp"
#include "joint_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#endif

namespace agi3sdk {

namespace {

std::int64_t SteadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

#ifdef AGI3SDK_WITH_AIMRT
namespace {

std::int64_t SystemNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::int64_t MonotonicNowNs() {
  return SteadyNowNs();
}

template <class Message>
std::int64_t MessageStampNs(const Message& message) {
  const auto value =
      static_cast<std::int64_t>(message.header.stamp.sec) * 1'000'000'000LL +
      static_cast<std::int64_t>(message.header.stamp.nanosec);
  // State receive timestamps are compared in the wall-clock domain.
  return value > 0 ? value : SystemNowNs();
}

template <std::size_t N>
bool DecodeJointGroup(const joint_msgs::msg::JointState& message,
                      const std::array<const char*, N>& expected,
                      JointGroupSample<N>& output) {
  if (message.joints.size() != N) return false;
  output.timestamp_ns = MessageStampNs(message);
  for (std::size_t target = 0; target < N; ++target) {
    bool found = false;
    for (const auto& joint : message.joints) {
      if (joint.name != expected[target]) continue;
      output.q[target] = joint.position;
      output.dq[target] = joint.velocity;
      output.tau_est[target] = joint.effort;
      found = true;
      break;
    }
    if (!found) return false;
  }
  return true;
}

ImuSample DecodeImu(const sensor_msgs::msg::Imu& message) {
  ImuSample output;
  output.timestamp_ns = MessageStampNs(message);
  output.quaternion_wxyz = {
      message.orientation.w, message.orientation.x,
      message.orientation.y, message.orientation.z};
  output.gyro = {
      message.angular_velocity.x, message.angular_velocity.y,
      message.angular_velocity.z};
  output.acceleration = {
      message.linear_acceleration.x, message.linear_acceleration.y,
      message.linear_acceleration.z};
  return output;
}

template <std::size_t N>
void FillJointCommand(const HardwareCommand& source, std::size_t offset,
                      const std::array<const char*, N>& names,
                      std::uint32_t sequence, std::int64_t timestamp_ns,
                      joint_msgs::msg::JointCommand& output) {
  output.header.stamp.sec =
      static_cast<std::int32_t>(timestamp_ns / 1'000'000'000LL);
  output.header.stamp.nanosec =
      static_cast<std::uint32_t>(timestamp_ns % 1'000'000'000LL);
  output.joints.resize(N);
  for (std::size_t index = 0; index < N; ++index) {
    const auto flat = offset + index;
    auto& joint = output.joints[index];
    joint.name = names[index];
    joint.sequence = sequence;
    joint.position = source.q_des[flat];
    joint.velocity = source.dq_des[flat];
    joint.effort = source.tau_ff[flat];
    joint.stiffness = source.kp[flat];
    joint.damping = source.kd[flat];
  }
}

void AnchorJointTypes() {
  (void)ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
      rosidl_typesupport_fastrtps_cpp, joint_msgs, msg, JointCommand)();
  (void)ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
      rosidl_typesupport_fastrtps_cpp, joint_msgs, msg, JointState)();
}

}  // namespace
#endif

class AimrtTransport::Impl {
 public:
  Impl(RobotIO& io, AimrtTransportConfig config)
      : io_(io), config_(std::move(config)),
        assembler_(config_.maximum_sync_skew_ns) {
#ifndef AGI3SDK_WITH_AIMRT
    throw std::runtime_error("agi3sdk was built without AimRT support");
#endif
  }

  bool Start() {
#ifdef AGI3SDK_WITH_AIMRT
    if (started_) return true;
    if (config_.config_path.empty()) return false;
    try {
      aimrt_options_.cfg_file_path = config_.config_path;
      aimrt_core_.Initialize(aimrt_options_);
      module_ = aimrt::CoreRef(
          aimrt_core_.GetModuleManager().CreateModule("Agi3SdkModule"));
      if (!RegisterChannels()) {
        aimrt_core_.Shutdown();
        return false;
      }
      shutdown_future_ = aimrt_core_.AsyncStart();
      started_ = true;
      return true;
    } catch (...) {
      try {
        aimrt_core_.Shutdown();
      } catch (...) {
      }
      return false;
    }
#else
    return false;
#endif
  }

  void Stop() {
#ifdef AGI3SDK_WITH_AIMRT
    if (!started_) return;
    try {
      aimrt_core_.Shutdown();
    } catch (...) {
    }
    if (shutdown_future_.valid()) {
      try {
        shutdown_future_.get();
      } catch (...) {
      }
    }
    started_ = false;
#endif
  }

  bool WriteCommand(const HardwareCommand& command) {
#ifdef AGI3SDK_WITH_AIMRT
    std::lock_guard lock(command_mutex_);
    if (!started_ || !config_.publish_enabled || !waist_publisher_ ||
        !neck_publisher_ || !arm_publisher_ || !leg_publisher_) {
      return false;
    }
    try {
      const auto sequence =
          command_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
      // body-drive uses the same monotonic timestamp domain as the vendor
      // AimRT deployment path. Wall-clock epoch timestamps are not valid here.
      const auto timestamp_ns = MonotonicNowNs();
      joint_msgs::msg::JointCommand waist;
      joint_msgs::msg::JointCommand neck;
      joint_msgs::msg::JointCommand arms;
      joint_msgs::msg::JointCommand legs;
      FillJointCommand(command, 0, kWaistJointNames, sequence, timestamp_ns,
                       waist);
      FillJointCommand(command, 3, kNeckJointNames, sequence, timestamp_ns,
                       neck);
      FillJointCommand(command, 5, kArmJointNames, sequence, timestamp_ns,
                       arms);
      FillJointCommand(command, 19, kLegJointNames, sequence, timestamp_ns,
                       legs);
      waist_publisher_->Publish(waist);
      neck_publisher_->Publish(neck);
      arm_publisher_->Publish(arms);
      leg_publisher_->Publish(legs);
      command_batches_.fetch_add(1, std::memory_order_relaxed);
      return true;
    } catch (...) {
      return false;
    }
#else
    (void)command;
    return false;
#endif
  }

  AimrtTransportStats Stats() const {
    AimrtTransportStats result;
    result.state_messages = state_messages_.load(std::memory_order_relaxed);
    result.invalid_state_messages =
        invalid_state_messages_.load(std::memory_order_relaxed);
    result.synchronized_frames =
        synchronized_frames_.load(std::memory_order_relaxed);
    result.command_batches = command_batches_.load(std::memory_order_relaxed);
    const auto now_ns = SteadyNowNs();
    std::int64_t oldest_source = 0;
    std::int64_t latest_source = 0;
    bool source_complete = true;
    for (std::size_t index = 0; index < kAimrtStateStreamCount; ++index) {
      result.stream_messages[index] =
          stream_messages_[index].load(std::memory_order_relaxed);
      result.stream_invalid_messages[index] =
          stream_invalid_messages_[index].load(std::memory_order_relaxed);
      const auto received =
          stream_received_at_ns_[index].load(std::memory_order_relaxed);
      const auto valid =
          stream_valid_at_ns_[index].load(std::memory_order_relaxed);
      const auto source =
          stream_source_timestamp_ns_[index].load(std::memory_order_relaxed);
      result.stream_receive_age_ns[index] =
          received > 0 ? std::max<std::int64_t>(0, now_ns - received) : -1;
      result.stream_valid_age_ns[index] =
          valid > 0 ? std::max<std::int64_t>(0, now_ns - valid) : -1;
      result.stream_source_timestamp_ns[index] = source;
      if (source <= 0) {
        source_complete = false;
      } else if (oldest_source == 0) {
        oldest_source = latest_source = source;
      } else {
        oldest_source = std::min(oldest_source, source);
        latest_source = std::max(latest_source, source);
      }
    }
    result.latest_source_skew_ns =
        source_complete ? latest_source - oldest_source : -1;
    return result;
  }

 private:
#ifdef AGI3SDK_WITH_AIMRT
  using JointPublisher =
      aimrt::channel::PublisherProxy<joint_msgs::msg::JointCommand>;

  bool RegisterChannels() {
    AnchorJointTypes();
    auto channel = module_.GetChannelHandle();
    if (config_.publish_enabled) {
      auto make_publisher = [&channel](std::string_view topic) {
        auto reference = channel.GetPublisher(topic);
        if (!aimrt::channel::RegisterPublishType<
                joint_msgs::msg::JointCommand>(reference)) {
          return std::unique_ptr<JointPublisher>{};
        }
        return std::make_unique<JointPublisher>(reference);
      };
      waist_publisher_ = make_publisher(config_.waist_command_topic);
      neck_publisher_ = make_publisher(config_.neck_command_topic);
      arm_publisher_ = make_publisher(config_.arm_command_topic);
      leg_publisher_ = make_publisher(config_.leg_command_topic);
      if (!waist_publisher_ || !neck_publisher_ || !arm_publisher_ ||
          !leg_publisher_) {
        return false;
      }
    }

    const auto subscribe_joint = [&channel](std::string_view topic, auto fn) {
      return aimrt::channel::Subscribe<joint_msgs::msg::JointState>(
          channel.GetSubscriber(topic), std::move(fn));
    };
    if (!subscribe_joint(
            config_.waist_state_topic,
            [this](const auto& message) { OnWaist(message); }) ||
        !subscribe_joint(
            config_.neck_state_topic,
            [this](const auto& message) { OnNeck(message); }) ||
        !subscribe_joint(
            config_.arm_state_topic,
            [this](const auto& message) { OnArms(message); }) ||
        !subscribe_joint(
            config_.leg_state_topic,
            [this](const auto& message) { OnLegs(message); })) {
      return false;
    }
    if (!aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(
            channel.GetSubscriber(config_.pelvis_imu_topic),
            [this](const auto& message) { OnPelvisImu(message); }) ||
        !aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(
            channel.GetSubscriber(config_.torso_imu_topic),
            [this](const auto& message) { OnTorsoImu(message); })) {
      return false;
    }
    return true;
  }

  void TryPublishState() {
    const auto state = assembler_.TryAssemble();
    if (state && io_.PublishState(*state)) {
      synchronized_frames_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  template <std::size_t N, class Update>
  void OnJoint(const std::shared_ptr<const joint_msgs::msg::JointState>& message,
               const std::array<const char*, N>& names, std::size_t stream,
               Update update) {
    state_messages_.fetch_add(1, std::memory_order_relaxed);
    stream_messages_[stream].fetch_add(1, std::memory_order_relaxed);
    stream_received_at_ns_[stream].store(
        SteadyNowNs(), std::memory_order_relaxed);
    JointGroupSample<N> sample;
    if (!message || !DecodeJointGroup(*message, names, sample)) {
      invalid_state_messages_.fetch_add(1, std::memory_order_relaxed);
      stream_invalid_messages_[stream].fetch_add(1, std::memory_order_relaxed);
      return;
    }
    stream_valid_at_ns_[stream].store(SteadyNowNs(), std::memory_order_relaxed);
    stream_source_timestamp_ns_[stream].store(
        sample.timestamp_ns, std::memory_order_relaxed);
    (assembler_.*update)(sample);
    TryPublishState();
  }

  void OnWaist(
      const std::shared_ptr<const joint_msgs::msg::JointState>& message) {
    OnJoint(message, kWaistJointNames, 0, &StateAssembler::UpdateWaist);
  }
  void OnNeck(
      const std::shared_ptr<const joint_msgs::msg::JointState>& message) {
    OnJoint(message, kNeckJointNames, 1, &StateAssembler::UpdateNeck);
  }
  void OnArms(
      const std::shared_ptr<const joint_msgs::msg::JointState>& message) {
    OnJoint(message, kArmJointNames, 2, &StateAssembler::UpdateArms);
  }
  void OnLegs(
      const std::shared_ptr<const joint_msgs::msg::JointState>& message) {
    OnJoint(message, kLegJointNames, 3, &StateAssembler::UpdateLegs);
  }
  void OnPelvisImu(const std::shared_ptr<const sensor_msgs::msg::Imu>& message) {
    state_messages_.fetch_add(1, std::memory_order_relaxed);
    stream_messages_[4].fetch_add(1, std::memory_order_relaxed);
    stream_received_at_ns_[4].store(SteadyNowNs(), std::memory_order_relaxed);
    if (!message) {
      invalid_state_messages_.fetch_add(1, std::memory_order_relaxed);
      stream_invalid_messages_[4].fetch_add(1, std::memory_order_relaxed);
      return;
    }
    const auto sample = DecodeImu(*message);
    stream_valid_at_ns_[4].store(SteadyNowNs(), std::memory_order_relaxed);
    stream_source_timestamp_ns_[4].store(
        sample.timestamp_ns, std::memory_order_relaxed);
    assembler_.UpdatePelvisImu(sample);
    TryPublishState();
  }
  void OnTorsoImu(const std::shared_ptr<const sensor_msgs::msg::Imu>& message) {
    state_messages_.fetch_add(1, std::memory_order_relaxed);
    stream_messages_[5].fetch_add(1, std::memory_order_relaxed);
    stream_received_at_ns_[5].store(SteadyNowNs(), std::memory_order_relaxed);
    if (!message) {
      invalid_state_messages_.fetch_add(1, std::memory_order_relaxed);
      stream_invalid_messages_[5].fetch_add(1, std::memory_order_relaxed);
      return;
    }
    const auto sample = DecodeImu(*message);
    stream_valid_at_ns_[5].store(SteadyNowNs(), std::memory_order_relaxed);
    stream_source_timestamp_ns_[5].store(
        sample.timestamp_ns, std::memory_order_relaxed);
    assembler_.UpdateTorsoImu(sample);
    TryPublishState();
  }
#endif

  RobotIO& io_;
  AimrtTransportConfig config_;
  StateAssembler assembler_;
  std::atomic<std::uint64_t> state_messages_{0};
  std::atomic<std::uint64_t> invalid_state_messages_{0};
  std::atomic<std::uint64_t> synchronized_frames_{0};
  std::atomic<std::uint64_t> command_batches_{0};
  std::array<std::atomic<std::uint64_t>, kAimrtStateStreamCount>
      stream_messages_{};
  std::array<std::atomic<std::uint64_t>, kAimrtStateStreamCount>
      stream_invalid_messages_{};
  std::array<std::atomic<std::int64_t>, kAimrtStateStreamCount>
      stream_received_at_ns_{};
  std::array<std::atomic<std::int64_t>, kAimrtStateStreamCount>
      stream_valid_at_ns_{};
  std::array<std::atomic<std::int64_t>, kAimrtStateStreamCount>
      stream_source_timestamp_ns_{};
  std::atomic<std::uint32_t> command_sequence_{0};
  std::mutex command_mutex_;
  bool started_ = false;
#ifdef AGI3SDK_WITH_AIMRT
  aimrt::runtime::core::AimRTCore aimrt_core_;
  aimrt::runtime::core::AimRTCore::Options aimrt_options_{};
  aimrt::CoreRef module_{};
  std::future<void> shutdown_future_{};
  std::unique_ptr<JointPublisher> waist_publisher_;
  std::unique_ptr<JointPublisher> neck_publisher_;
  std::unique_ptr<JointPublisher> arm_publisher_;
  std::unique_ptr<JointPublisher> leg_publisher_;
#endif
};

AimrtTransport::AimrtTransport(RobotIO& io, AimrtTransportConfig config)
    : impl_(std::make_unique<Impl>(io, std::move(config))) {}

AimrtTransport::~AimrtTransport() { impl_->Stop(); }

bool AimrtTransport::Start() { return impl_->Start(); }

void AimrtTransport::Stop() { impl_->Stop(); }

bool AimrtTransport::WriteCommand(const HardwareCommand& command) {
  return impl_->WriteCommand(command);
}

AimrtTransportStats AimrtTransport::Stats() const { return impl_->Stats(); }

}  // namespace agi3sdk
