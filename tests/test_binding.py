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
        )
        state = io.read_state(timeout_s=0.01)
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
