from pathlib import Path
import time

import numpy as np
import pytest

from agi3sdk import JointCommand
from agi3sdk import A3RobotIO, RobotState, StateStaleError


ROOT = Path(__file__).resolve().parents[1]
SDK_LIBRARY = ROOT / "build/native/libagi3sdk.so"


def test_robot_state_owns_and_normalizes_policy_snapshot():
    source = np.arange(29, dtype=np.float64)
    state = RobotState(
        sequence=1,
        timestamp_ns=20_000_000,
        gyro_b=np.zeros(3),
        quaternion_wxyz=np.asarray([2.0, 0.0, 0.0, 0.0]),
        joint_pos=source,
        joint_vel=np.zeros(29),
        joint_tau_est=np.zeros(29),
    )
    source.fill(-1)
    assert state.joint_pos[1] == 1
    assert state.quaternion_wxyz.tolist() == [1.0, 0.0, 0.0, 0.0]
    assert not state.has_torso_imu
    assert state.torso_quaternion_wxyz.tolist() == [1.0, 0.0, 0.0, 0.0]
    assert state.pelvis_acceleration_b is None


def test_robot_state_exposes_normalized_torso_imu_snapshot():
    state = RobotState(
        sequence=2,
        timestamp_ns=40_000_000,
        gyro_b=np.zeros(3),
        quaternion_wxyz=np.asarray([1.0, 0.0, 0.0, 0.0]),
        joint_pos=np.zeros(29),
        joint_vel=np.zeros(29),
        joint_tau_est=np.zeros(29),
        has_torso_imu=True,
        torso_gyro_b=np.asarray([1.0, 2.0, 3.0]),
        torso_quaternion_wxyz=np.asarray([2.0, 0.0, 0.0, 0.0]),
        torso_acceleration_b=np.asarray([0.1, 0.2, 9.7]),
    )

    assert state.has_torso_imu
    assert state.torso_gyro_b.tolist() == [1.0, 2.0, 3.0]
    assert state.torso_quaternion_wxyz.tolist() == [1.0, 0.0, 0.0, 0.0]
    assert state.torso_acceleration_b.tolist() == [0.1, 0.2, 9.7]


def test_cpp_sdk_python_binding_uses_canonical_mapping():
    assert SDK_LIBRARY.is_file(), "build the agi3sdk CMake target before pytest"
    io = A3RobotIO(
        library_path=SDK_LIBRARY, read_only=False, command_timeout_ms=1000,
    )
    io._install_memory_command_writer_for_test()
    io.start()
    try:
        hardware_q = np.arange(31, dtype=np.float64)
        io._inject_hardware_state_for_test(
            4,
            hardware_q,
            torso_quaternion_wxyz=np.asarray([1.0, 0.0, 0.05, 0.0]),
            torso_gyro=np.asarray([0.1, 0.2, 0.3]),
            torso_acceleration=np.asarray([0.0, 0.0, 9.7]),
            pelvis_acceleration=np.asarray([0.3, -0.2, 9.6]),
        )
        state = io.read_state(timeout_s=0.01)
        assert state.pelvis_acceleration_b.tolist() == pytest.approx([0.3, -0.2, 9.6])
        assert state.joint_pos.tolist() == (
            list(range(19, 31)) + list(range(0, 3)) + list(range(5, 19))
        )
        assert state.has_torso_imu
        assert state.torso_gyro_b.tolist() == pytest.approx([0.1, 0.2, 0.3])
        zeros = np.zeros(29)
        io.write_command(
            JointCommand(zeros, zeros, zeros, zeros, zeros), state.sequence
        )
    finally:
        io.close()


def test_cpp_sdk_exposes_stale_snapshot_as_typed_error():
    io = A3RobotIO(
        library_path=SDK_LIBRARY, read_only=False,
        state_timeout_ms=5, command_timeout_ms=1000,
    )
    io._install_memory_command_writer_for_test()
    io.start()
    try:
        io._inject_hardware_state_for_test(1, np.zeros(31))
        state = io.read_state(timeout_s=0.01)
        time.sleep(0.01)
        zeros = np.zeros(29)
        with pytest.raises(StateStaleError):
            io.write_command(
                JointCommand(zeros, zeros, zeros, zeros, zeros),
                state.sequence,
            )
    finally:
        io.close()


def test_cpp_sdk_neck_preset_ramps_from_measured_position():
    from agi3sdk import SdkError
    from agi3sdk import sdk as binding

    with pytest.raises(SdkError):
        A3RobotIO(library_path=SDK_LIBRARY, neck_mode="fixed")
    out_of_range = A3RobotIO(
        library_path=SDK_LIBRARY, neck_mode="preset", neck_preset_q=(0.0, 0.5),
    )
    with pytest.raises(SdkError):
        out_of_range.start()
    out_of_range.close()

    io = A3RobotIO(
        library_path=SDK_LIBRARY, read_only=False, command_timeout_ms=1000,
        neck_mode="preset", neck_preset_q=(0.0, 0.17),
        neck_preset_max_velocity=1.0e6,
    )
    neck = []
    io._test_writer = binding._CommandWriter(
        lambda command, _user: neck.append(tuple(command.contents.q_des[3:5])) or 1
    )
    assert io._library.agi3sdk_set_command_writer(io._handle, io._test_writer, None)
    io.start()
    try:
        hardware_q = np.zeros(31)
        hardware_q[3:5] = (0.2, -0.3)
        io._inject_hardware_state_for_test(1, hardware_q)
        state = io.read_state(timeout_s=0.01)
        zeros = np.zeros(29)
        for _ in range(2):
            io.write_command(JointCommand(zeros, zeros, zeros, zeros, zeros), state.sequence)
            time.sleep(0.005)
    finally:
        io.close()
    assert neck[0] == pytest.approx((0.2, -0.3))
    assert neck[1] == pytest.approx((0.0, 0.17))


def test_read_hardware_state_returns_the_measured_neck():
    from agi3sdk import HardwareState

    io = A3RobotIO(library_path=SDK_LIBRARY, read_only=True)
    assert io.abi_version == 6 and io.has_hardware_state
    io.start()
    try:
        assert io.read_hardware_state() is None             # before the first state
        q = np.arange(31, dtype=np.float64) * 0.01
        q[3:5] = (0.05, -0.12)                              # head_yaw, head_pitch
        dq = np.zeros(31)
        dq[3:5] = (0.3, -0.4)
        io._inject_hardware_state_for_test(5, q, dq, pelvis_acceleration=(0.1, 0.2, 9.7))
        state = io.read_hardware_state()
        assert isinstance(state, HardwareState)
        assert state.sequence == 5 and state.timestamp_ns == 5 * 20_000_000 and state.sync_complete
        np.testing.assert_array_equal(state.q, q)
        np.testing.assert_array_equal(state.neck_q, (0.05, -0.12))
        np.testing.assert_array_equal(state.neck_dq, (0.3, -0.4))
        np.testing.assert_array_equal(state.waist_q, q[0:3])
        np.testing.assert_array_equal(state.pelvis_acceleration, (0.1, 0.2, 9.7))
        state.q[3] = 99.0                                   # an owned copy
        assert io.read_hardware_state().q[3] == 0.05
        # the policy view is unchanged (no neck, 29-D)
        policy = io.read_state(timeout_s=0.01)
        assert policy.joint_pos.shape == (29,) and policy.joint_pos[12] == q[0]
    finally:
        io.close()
    with pytest.raises(Exception):
        io.read_hardware_state()                            # closed


def test_abi5_library_still_loads_without_the_hardware_state_reader():
    """Rollback safety: point AGI3SDK_ABI5_LIBRARY at an ABI-5 build (skipped otherwise)."""
    import os
    from agi3sdk import SdkError

    path = os.environ.get("AGI3SDK_ABI5_LIBRARY")
    if not path:
        pytest.skip("AGI3SDK_ABI5_LIBRARY not set")
    io = A3RobotIO(library_path=path, read_only=True)
    try:
        assert io.abi_version == 5 and not io.has_hardware_state
        with pytest.raises(SdkError, match="ABI >= 6"):
            io.read_hardware_state()
    finally:
        io.close()
