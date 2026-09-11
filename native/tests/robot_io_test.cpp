#include "agi3sdk/c_api.h"
#include "agi3sdk/robot_io.hpp"
#include "agi3sdk/state_assembler.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

namespace {

agi3sdk::RobotState MakeState(std::uint64_t sequence) {
  agi3sdk::RobotState state;
  state.sequence = sequence;
  state.timestamp_ns = 123456;
  state.sync_complete = true;
  for (std::size_t i = 0; i < agi3sdk::kHardwareDof; ++i) {
    state.q[i] = 100.0 + static_cast<double>(i);
    state.dq[i] = 200.0 + static_cast<double>(i);
    state.tau_est[i] = 300.0 + static_cast<double>(i);
  }
  state.pelvis_gyro = {1.0, 2.0, 3.0};
  state.has_torso_imu = true;
  state.torso_quaternion_wxyz = {0.99, 0.01, 0.02, 0.03};
  state.torso_gyro = {4.0, 5.0, 6.0};
  state.torso_acceleration = {0.1, 0.2, 9.7};
  return state;
}

agi3sdk::PolicyCommand MakeCommand() {
  agi3sdk::PolicyCommand command;
  for (std::size_t i = 0; i < agi3sdk::kPolicyDof; ++i) {
    command.q_des[i] = 10.0 + static_cast<double>(i);
    command.dq_des[i] = 20.0 + static_cast<double>(i);
    command.kp[i] = 30.0 + static_cast<double>(i);
    command.kd[i] = 2.0;
    command.tau_ff[i] = 40.0 + static_cast<double>(i);
  }
  return command;
}

void TestMappingAndGuards() {
  agi3sdk::Config config;
  config.read_only = false;
  config.command_timeout = std::chrono::milliseconds(1000);
  agi3sdk::RobotIO io(config, [](const auto&) { return true; });
  assert(io.Start());
  assert(io.PublishState(MakeState(7)));
  assert(!io.PublishState(MakeState(7)));

  const auto policy = io.WaitNextState(0, std::chrono::milliseconds(1));
  assert(policy && policy->sequence == 7);
  assert(policy->q[0] == 119.0);   // left leg
  assert(policy->q[11] == 130.0); // right leg
  assert(policy->q[12] == 100.0); // waist
  assert(policy->q[15] == 105.0); // left arm
  assert(policy->has_torso_imu);
  assert(policy->torso_gyro[0] == 4.0 && policy->torso_gyro[1] == 5.0);

  auto command = MakeCommand();
  assert(io.SendPolicyCommand(command, 6) ==
         agi3sdk::SendResult::kStateSequenceMismatch);
  // A newer AimRT callback arriving during inference must not invalidate the
  // immutable snapshot already delivered to the policy.
  assert(io.PublishState(MakeState(8)));
  assert(io.SendPolicyCommand(command, 7) == agi3sdk::SendResult::kOk);
  const auto hardware = io.LastHardwareCommand();
  assert(hardware);
  assert(hardware->q_des[19] == 10.0);
  assert(hardware->q_des[0] == 22.0);
  assert(hardware->q_des[5] == 25.0);
  assert(hardware->q_des[3] == 103.0); // neck holds measured position
  assert(hardware->q_des[4] == 104.0);
  assert(hardware->kp[3] == config.neck_kp);

  command.kp[0] = -1.0;
  assert(io.SendPolicyCommand(command, 7) ==
         agi3sdk::SendResult::kInvalidCommand);
  io.Stop();
  const auto stopped = io.LastHardwareCommand();
  assert(stopped && stopped->safe_damping);
  assert(stopped->kp[0] == 0.0 && stopped->kd[0] == config.safe_damping_kd);
}

void TestWatchdog() {
  agi3sdk::Config config;
  config.read_only = false;
  config.command_timeout = std::chrono::milliseconds(20);
  agi3sdk::RobotIO io(config, [](const auto&) { return true; });
  assert(io.Start());
  assert(io.PublishState(MakeState(1)));
  assert(io.WaitNextState(0, std::chrono::milliseconds(1)));
  assert(io.SendPolicyCommand(MakeCommand(), 1) == agi3sdk::SendResult::kOk);
  std::this_thread::sleep_for(std::chrono::milliseconds(45));
  const auto command = io.LastHardwareCommand();
  assert(command && command->safe_damping);
  assert(command->kp[0] == 0.0);
  assert(command->kd[0] == config.safe_damping_kd);
  assert(io.watchdog_trip_count() == 1);
  io.Stop();
}

void TestDeliveredStateFreshness() {
  agi3sdk::Config config;
  config.read_only = false;
  config.state_timeout = std::chrono::milliseconds(10);
  config.command_timeout = std::chrono::milliseconds(1000);
  agi3sdk::RobotIO io(config, [](const auto&) { return true; });
  assert(io.Start());
  assert(io.PublishState(MakeState(1)));
  assert(io.WaitNextState(0, std::chrono::milliseconds(1)));
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  assert(io.SendPolicyCommand(MakeCommand(), 1) ==
         agi3sdk::SendResult::kStateStale);
  io.Stop();
}

void TestCApi() {
  assert(agi3sdk_abi_version() == AGI3SDK_ABI_VERSION);
  agi3sdk_config config{50, 1000, 20.0, 1.0, 3.0, 0};
  auto* handle = agi3sdk_create(&config);
  assert(handle && agi3sdk_start(handle));
  agi3sdk_hardware_state input{};
  input.sequence = 9;
  input.sync_complete = 1;
  input.pelvis_quaternion_wxyz[0] = 1.0;
  input.pelvis_acceleration[2] = 9.81;
  input.has_torso_imu = 1;
  input.torso_quaternion_wxyz[0] = 0.99;
  input.torso_quaternion_wxyz[2] = 0.02;
  input.torso_gyro[1] = 5.0;
  input.torso_acceleration[2] = 9.7;
  for (int i = 0; i < AGI3SDK_HARDWARE_DOF; ++i) input.q[i] = i;
  assert(agi3sdk_publish_hardware_state(handle, &input));
  agi3sdk_policy_state output{};
  assert(agi3sdk_wait_next_state(handle, 0, 1, &output));
  assert(output.q[0] == 19.0 && output.q[12] == 0.0);
  assert(output.has_torso_imu == 1);
  assert(output.torso_gyro[1] == 5.0);
  agi3sdk_stop(handle);
  agi3sdk_destroy(handle);
}

void TestSixStreamAssembler() {
  agi3sdk::StateAssembler assembler(5'000'000);
  agi3sdk::JointGroupSample<3> waist;
  agi3sdk::JointGroupSample<2> neck;
  agi3sdk::JointGroupSample<14> arms;
  agi3sdk::JointGroupSample<12> legs;
  agi3sdk::ImuSample pelvis;
  agi3sdk::ImuSample torso;
  waist.timestamp_ns = 10'000'000;
  neck.timestamp_ns = 11'000'000;
  arms.timestamp_ns = 12'000'000;
  legs.timestamp_ns = 13'000'000;
  pelvis.timestamp_ns = 14'000'000;
  torso.timestamp_ns = 15'000'000;
  waist.q = {1.0, 2.0, 3.0};
  neck.q = {4.0, 5.0};
  legs.q[0] = 19.0;
  assembler.UpdateWaist(waist);
  assembler.UpdateNeck(neck);
  assembler.UpdateArms(arms);
  assembler.UpdateLegs(legs);
  assembler.UpdatePelvisImu(pelvis);
  assert(!assembler.TryAssemble());
  assembler.UpdateTorsoImu(torso);
  const auto state = assembler.TryAssemble();
  assert(state && state->sequence == 1 && state->sync_skew_ns == 5'000'000);
  assert(state->q[0] == 1.0 && state->q[3] == 4.0 && state->q[19] == 19.0);
  assert(!assembler.TryAssemble());

  // A fast topic alone must not generate duplicate frames from stale samples.
  waist.timestamp_ns = 16'000'000;
  assembler.UpdateWaist(waist);
  assert(!assembler.TryAssemble());
  neck.timestamp_ns = 16'000'000;
  arms.timestamp_ns = 16'000'000;
  legs.timestamp_ns = 16'000'000;
  pelvis.timestamp_ns = 16'000'000;
  torso.timestamp_ns = 16'000'000;
  assembler.UpdateNeck(neck);
  assembler.UpdateArms(arms);
  assembler.UpdateLegs(legs);
  assembler.UpdatePelvisImu(pelvis);
  assembler.UpdateTorsoImu(torso);
  const auto next = assembler.TryAssemble();
  assert(next && next->sequence == 2 && next->sync_skew_ns == 0);
}

}  // namespace

int main() {
  TestMappingAndGuards();
  TestWatchdog();
  TestDeliveredStateFreshness();
  TestCApi();
  TestSixStreamAssembler();
}
