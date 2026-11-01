"""Dependency-free ctypes binding for the C++ agi3sdk core."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
import os
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np

from .types import JointCommand
from .types import RobotState

POLICY_DOF = 29
HARDWARE_DOF = 31
SDK_ABI_VERSION = 6
# Library ABIs this binding loads. ABI 6 only adds agi3sdk_copy_last_hardware_state;
# an ABI-5 library still works, without read_hardware_state() (rollback-safe).
SUPPORTED_ABI_VERSIONS = (5, 6)
HARDWARE_STATE_ABI = 6
# Values of agi3sdk_config.neck_mode (agi3sdk::NeckMode).
_NECK_MODES = {"hold_measured": 0, "preset": 1}
STATE_STREAM_NAMES = (
    "waist", "neck", "arms", "legs", "pelvis_imu", "torso_imu",
)


class SdkError(RuntimeError):
    pass


class StateStaleError(SdkError):
    """The policy command was computed from an expired delivered snapshot."""


@dataclass(frozen=True)
class HardwareState:
    """Newest assembled 31-D hardware snapshot, body-drive order:
    waist 3 (yaw, roll, pitch), neck 2 (head_yaw, head_pitch), arms 14, legs 12."""

    sequence: int
    timestamp_ns: int
    sync_skew_ns: int
    sync_complete: bool
    q: np.ndarray
    dq: np.ndarray
    tau_est: np.ndarray
    pelvis_quaternion_wxyz: np.ndarray
    pelvis_gyro: np.ndarray
    pelvis_acceleration: np.ndarray
    has_torso_imu: bool
    torso_quaternion_wxyz: np.ndarray
    torso_gyro: np.ndarray
    torso_acceleration: np.ndarray

    @property
    def waist_q(self) -> np.ndarray:
        return self.q[0:3]

    @property
    def neck_q(self) -> np.ndarray:
        return self.q[3:5]

    @property
    def neck_dq(self) -> np.ndarray:
        return self.dq[3:5]


class _Config(ctypes.Structure):
    _fields_ = [
        ("state_timeout_ms", ctypes.c_int),
        ("command_timeout_ms", ctypes.c_int),
        ("neck_kp", ctypes.c_double),
        ("neck_kd", ctypes.c_double),
        ("safe_damping_kd", ctypes.c_double),
        ("read_only", ctypes.c_int),
        ("neck_mode", ctypes.c_int),
        ("neck_preset_q", ctypes.c_double * 2),
        ("neck_preset_max_velocity", ctypes.c_double),
    ]


class _AimrtConfig(ctypes.Structure):
    _fields_ = [
        ("config_path", ctypes.c_char_p),
        ("publish_enabled", ctypes.c_int),
        ("maximum_sync_skew_ns", ctypes.c_int64),
        ("waist_state_topic", ctypes.c_char_p),
        ("neck_state_topic", ctypes.c_char_p),
        ("arm_state_topic", ctypes.c_char_p),
        ("leg_state_topic", ctypes.c_char_p),
        ("pelvis_imu_topic", ctypes.c_char_p),
        ("torso_imu_topic", ctypes.c_char_p),
        ("waist_command_topic", ctypes.c_char_p),
        ("neck_command_topic", ctypes.c_char_p),
        ("arm_command_topic", ctypes.c_char_p),
        ("leg_command_topic", ctypes.c_char_p),
    ]


class _AimrtStats(ctypes.Structure):
    _fields_ = [
        ("state_messages", ctypes.c_uint64),
        ("invalid_state_messages", ctypes.c_uint64),
        ("synchronized_frames", ctypes.c_uint64),
        ("command_batches", ctypes.c_uint64),
        ("stream_messages", ctypes.c_uint64 * len(STATE_STREAM_NAMES)),
        ("stream_invalid_messages", ctypes.c_uint64 * len(STATE_STREAM_NAMES)),
        ("stream_receive_age_ns", ctypes.c_int64 * len(STATE_STREAM_NAMES)),
        ("stream_valid_age_ns", ctypes.c_int64 * len(STATE_STREAM_NAMES)),
        ("stream_source_timestamp_ns", ctypes.c_int64 * len(STATE_STREAM_NAMES)),
        ("latest_source_skew_ns", ctypes.c_int64),
    ]


class _PolicyState(ctypes.Structure):
    _fields_ = [
        ("sequence", ctypes.c_uint64),
        ("timestamp_ns", ctypes.c_int64),
        ("sync_skew_ns", ctypes.c_int64),
        ("sync_complete", ctypes.c_int),
        ("q", ctypes.c_double * POLICY_DOF),
        ("dq", ctypes.c_double * POLICY_DOF),
        ("tau_est", ctypes.c_double * POLICY_DOF),
        ("quaternion_wxyz", ctypes.c_double * 4),
        ("gyro", ctypes.c_double * 3),
        ("acceleration", ctypes.c_double * 3),
        ("has_torso_imu", ctypes.c_int),
        ("torso_quaternion_wxyz", ctypes.c_double * 4),
        ("torso_gyro", ctypes.c_double * 3),
        ("torso_acceleration", ctypes.c_double * 3),
    ]


class _PolicyCommand(ctypes.Structure):
    _fields_ = [
        ("q_des", ctypes.c_double * POLICY_DOF),
        ("dq_des", ctypes.c_double * POLICY_DOF),
        ("kp", ctypes.c_double * POLICY_DOF),
        ("kd", ctypes.c_double * POLICY_DOF),
        ("tau_ff", ctypes.c_double * POLICY_DOF),
    ]


class _HardwareState(ctypes.Structure):
    _fields_ = [
        ("sequence", ctypes.c_uint64),
        ("timestamp_ns", ctypes.c_int64),
        ("sync_skew_ns", ctypes.c_int64),
        ("sync_complete", ctypes.c_int),
        ("q", ctypes.c_double * HARDWARE_DOF),
        ("dq", ctypes.c_double * HARDWARE_DOF),
        ("tau_est", ctypes.c_double * HARDWARE_DOF),
        ("pelvis_quaternion_wxyz", ctypes.c_double * 4),
        ("pelvis_gyro", ctypes.c_double * 3),
        ("pelvis_acceleration", ctypes.c_double * 3),
        ("has_torso_imu", ctypes.c_int),
        ("torso_quaternion_wxyz", ctypes.c_double * 4),
        ("torso_gyro", ctypes.c_double * 3),
        ("torso_acceleration", ctypes.c_double * 3),
    ]


class _HardwareCommand(ctypes.Structure):
    _fields_ = [
        ("q_des", ctypes.c_double * HARDWARE_DOF),
        ("dq_des", ctypes.c_double * HARDWARE_DOF),
        ("kp", ctypes.c_double * HARDWARE_DOF),
        ("kd", ctypes.c_double * HARDWARE_DOF),
        ("tau_ff", ctypes.c_double * HARDWARE_DOF),
        ("safe_damping", ctypes.c_int),
    ]


_CommandWriter = ctypes.CFUNCTYPE(
    ctypes.c_int, ctypes.POINTER(_HardwareCommand), ctypes.c_void_p,
)


def _copy(values, destination) -> None:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if array.size != len(destination):
        raise ValueError(f"expected {len(destination)} values, got {array.size}")
    for index, value in enumerate(array):
        destination[index] = value


def _library_candidates(explicit: str | Path | None):
    if explicit is not None:
        yield Path(explicit)
    configured = os.environ.get("AGI3SDK_LIBRARY")
    if configured:
        yield Path(configured)
    yield Path(__file__).resolve().parent / "lib" / "libagi3sdk.so"
    yield Path(__file__).resolve().parents[2] / "build" / "native" / "libagi3sdk.so"
    yield Path("/usr/local/lib/libagi3sdk.so")


def _load_library(path: str | Path | None):
    selected = next((candidate for candidate in _library_candidates(path)
                     if candidate.is_file()), None)
    if selected is None:
        raise SdkError(
            "libagi3sdk.so not found; build with `cmake -S . -B build && "
            "cmake --build build --target agi3sdk`, or set AGI3SDK_LIBRARY"
        )
    library = ctypes.CDLL(str(selected.resolve()))
    library.agi3sdk_abi_version.argtypes = []
    library.agi3sdk_abi_version.restype = ctypes.c_uint32
    actual_abi = int(library.agi3sdk_abi_version())
    if actual_abi not in SUPPORTED_ABI_VERSIONS:
        raise SdkError(
            "libagi3sdk.so has an incompatible C ABI version "
            f"(expected one of {SUPPORTED_ABI_VERSIONS}, got {actual_abi}); rebuild agi3sdk"
        )
    library.agi3sdk_loaded_abi = actual_abi
    library.agi3sdk_create.argtypes = [ctypes.POINTER(_Config)]
    library.agi3sdk_create.restype = ctypes.c_void_p
    library.agi3sdk_destroy.argtypes = [ctypes.c_void_p]
    library.agi3sdk_start.argtypes = [ctypes.c_void_p]
    library.agi3sdk_start.restype = ctypes.c_int
    library.agi3sdk_stop.argtypes = [ctypes.c_void_p]
    library.agi3sdk_configure_aimrt.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(_AimrtConfig),
    ]
    library.agi3sdk_configure_aimrt.restype = ctypes.c_int
    library.agi3sdk_get_aimrt_stats.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(_AimrtStats),
    ]
    library.agi3sdk_get_aimrt_stats.restype = ctypes.c_int
    library.agi3sdk_set_command_writer.argtypes = [
        ctypes.c_void_p, _CommandWriter, ctypes.c_void_p,
    ]
    library.agi3sdk_set_command_writer.restype = ctypes.c_int
    library.agi3sdk_wait_next_state.argtypes = [
        ctypes.c_void_p, ctypes.c_uint64, ctypes.c_int,
        ctypes.POINTER(_PolicyState),
    ]
    library.agi3sdk_wait_next_state.restype = ctypes.c_int
    library.agi3sdk_send_policy_command.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(_PolicyCommand), ctypes.c_uint64,
    ]
    library.agi3sdk_send_policy_command.restype = ctypes.c_int
    library.agi3sdk_watchdog_trip_count.argtypes = [ctypes.c_void_p]
    library.agi3sdk_watchdog_trip_count.restype = ctypes.c_uint64
    library.agi3sdk_publish_hardware_state.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(_HardwareState),
    ]
    library.agi3sdk_publish_hardware_state.restype = ctypes.c_int
    if actual_abi >= HARDWARE_STATE_ABI:
        library.agi3sdk_copy_last_hardware_state.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(_HardwareState),
        ]
        library.agi3sdk_copy_last_hardware_state.restype = ctypes.c_int
    return library


class A3RobotIO:
    """Python policy-view facade over the C++ SDK.

    The target-specific AimRT adapter feeds synchronized 31-D snapshots into
    the same C++ RobotIO instance. This class never imports vendor messages.
    """

    def __init__(
        self,
        *,
        library_path: str | Path | None = None,
        read_only: bool = True,
        state_timeout_ms: int = 50,
        command_timeout_ms: int = 100,
        neck_kp: float = 20.0,
        neck_kd: float = 1.0,
        safe_damping_kd: float = 3.0,
        neck_mode: str = "hold_measured",
        neck_preset_q: Sequence[float] = (0.0, 0.0),
        neck_preset_max_velocity: float = 0.3,
        aimrt_config_path: str | Path | None = None,
        command_publish_enabled: bool = False,
        maximum_sync_skew_ns: int = 20_000_000,
        state_topics: Mapping[str, str] | None = None,
        command_topics: Mapping[str, str] | None = None,
    ) -> None:
        if command_publish_enabled and read_only:
            raise SdkError("a read-only SDK cannot enable command publication")
        if command_publish_enabled and aimrt_config_path is None:
            raise SdkError("command publication requires an AimRT config")
        if neck_mode not in _NECK_MODES:
            raise SdkError(
                f"neck_mode must be one of {sorted(_NECK_MODES)}, got {neck_mode!r}"
            )
        if len(neck_preset_q) != 2:
            raise SdkError("neck_preset_q must be [head_yaw, head_pitch]")
        self._library = _load_library(library_path)
        config = _Config(
            state_timeout_ms, command_timeout_ms, neck_kp, neck_kd,
            safe_damping_kd, int(read_only), _NECK_MODES[neck_mode],
            (ctypes.c_double * 2)(*neck_preset_q), neck_preset_max_velocity,
        )
        self._handle = self._library.agi3sdk_create(ctypes.byref(config))
        if not self._handle:
            raise SdkError("agi3sdk_create failed")
        self._aimrt_strings: list[bytes] = []
        if aimrt_config_path is not None:
            resolved = Path(aimrt_config_path).expanduser().resolve()
            if not resolved.is_file():
                self._library.agi3sdk_destroy(self._handle)
                self._handle = None
                raise SdkError(f"AimRT config not found: {resolved}")
            if state_topics is None or command_topics is None:
                self._library.agi3sdk_destroy(self._handle)
                self._handle = None
                raise SdkError(
                    "AimRT state_topics and command_topics must come from config"
                )
            topic_values = [
                state_topics[name]
                for name in ("waist", "neck", "arms", "legs", "pelvis_imu", "torso_imu")
            ] + [
                command_topics[name]
                for name in ("waist", "neck", "arms", "legs")
            ]
            if any(not value.startswith("/") for value in topic_values):
                self._library.agi3sdk_destroy(self._handle)
                self._handle = None
                raise SdkError("every AimRT topic must be an absolute name")
            self._aimrt_strings = [os.fsencode(resolved)] + [
                value.encode() for value in topic_values
            ]
            transport = _AimrtConfig(
                self._aimrt_strings[0],
                int(command_publish_enabled),
                int(maximum_sync_skew_ns),
                *self._aimrt_strings[1:],
            )
            if not self._library.agi3sdk_configure_aimrt(
                self._handle, ctypes.byref(transport)
            ):
                self._library.agi3sdk_destroy(self._handle)
                self._handle = None
                raise SdkError(
                    "failed to configure AimRT transport; verify the SDK was "
                    "built with AGI3SDK_WITH_AIMRT=ON"
                )
        self._last_sequence = 0
        self._started = False
        self._test_writer = None

    def start(self) -> None:
        if not self._library.agi3sdk_start(self._handle):
            raise SdkError("agi3sdk_start rejected its configuration")
        self._started = True

    def read_state(self, timeout_s: float | None = None) -> RobotState:
        if not self._started:
            raise SdkError("A3RobotIO is not started")
        timeout_ms = 30 if timeout_s is None else max(0, round(timeout_s * 1000))
        state = _PolicyState()
        received = self._library.agi3sdk_wait_next_state(
            self._handle, self._last_sequence, timeout_ms, ctypes.byref(state)
        )
        if not received:
            raise TimeoutError("timed out waiting for a new A3 state snapshot")
        self._last_sequence = int(state.sequence)
        return RobotState(
            sequence=int(state.sequence),
            timestamp_ns=int(state.timestamp_ns),
            gyro_b=np.ctypeslib.as_array(state.gyro).copy(),
            quaternion_wxyz=np.ctypeslib.as_array(state.quaternion_wxyz).copy(),
            joint_pos=np.ctypeslib.as_array(state.q).copy(),
            joint_vel=np.ctypeslib.as_array(state.dq).copy(),
            joint_tau_est=np.ctypeslib.as_array(state.tau_est).copy(),
            sync_complete=bool(state.sync_complete),
            sync_skew_ns=int(state.sync_skew_ns),
            has_torso_imu=bool(state.has_torso_imu),
            torso_gyro_b=np.ctypeslib.as_array(state.torso_gyro).copy(),
            torso_quaternion_wxyz=np.ctypeslib.as_array(
                state.torso_quaternion_wxyz
            ).copy(),
            torso_acceleration_b=np.ctypeslib.as_array(
                state.torso_acceleration
            ).copy(),
            pelvis_acceleration_b=np.ctypeslib.as_array(state.acceleration).copy(),
        )

    def write_command(self, command: JointCommand, state_sequence: int) -> None:
        packet = _PolicyCommand()
        for name in ("q_des", "dq_des", "kp", "kd", "tau_ff"):
            _copy(getattr(command, name), getattr(packet, name))
        result = self._library.agi3sdk_send_policy_command(
            self._handle, ctypes.byref(packet), state_sequence
        )
        if result != 0:
            messages = {
                1: "SDK not running or read-only",
                2: "no A3 state received",
                3: "state sequence mismatch",
                4: "A3 state is stale",
                5: "command contains invalid values",
                6: "A3 command transport failed",
            }
            message = messages.get(result, f"SDK send failed ({result})")
            if result == 4:
                raise StateStaleError(message)
            raise SdkError(message)

    @property
    def abi_version(self) -> int:
        """C ABI of the loaded libagi3sdk.so."""
        return int(self._library.agi3sdk_loaded_abi)

    @property
    def has_hardware_state(self) -> bool:
        """True when the library exports agi3sdk_copy_last_hardware_state (ABI >= 6)."""
        return self.abi_version >= HARDWARE_STATE_ABI

    def read_hardware_state(self) -> HardwareState | None:
        """Newest assembled 31-D hardware state (incl. the measured neck), read-only.

        Non-blocking; None before the first state. Raises SdkError on an ABI-5
        library, which has no such getter."""
        if not self.has_hardware_state:
            raise SdkError(
                f"read_hardware_state needs libagi3sdk ABI >= {HARDWARE_STATE_ABI} "
                f"(loaded ABI {self.abi_version})"
            )
        if not self._handle:
            raise SdkError("A3RobotIO is closed")
        state = _HardwareState()
        if not self._library.agi3sdk_copy_last_hardware_state(self._handle, ctypes.byref(state)):
            return None

        def array(values):
            return np.ctypeslib.as_array(values).copy()

        return HardwareState(
            sequence=int(state.sequence), timestamp_ns=int(state.timestamp_ns),
            sync_skew_ns=int(state.sync_skew_ns), sync_complete=bool(state.sync_complete),
            q=array(state.q), dq=array(state.dq), tau_est=array(state.tau_est),
            pelvis_quaternion_wxyz=array(state.pelvis_quaternion_wxyz),
            pelvis_gyro=array(state.pelvis_gyro),
            pelvis_acceleration=array(state.pelvis_acceleration),
            has_torso_imu=bool(state.has_torso_imu),
            torso_quaternion_wxyz=array(state.torso_quaternion_wxyz),
            torso_gyro=array(state.torso_gyro), torso_acceleration=array(state.torso_acceleration),
        )

    @property
    def watchdog_trip_count(self) -> int:
        return int(self._library.agi3sdk_watchdog_trip_count(self._handle))

    @property
    def aimrt_stats(self) -> dict[str, object]:
        stats = _AimrtStats()
        if not self._library.agi3sdk_get_aimrt_stats(
            self._handle, ctypes.byref(stats)
        ):
            return {}
        streams = {}
        for index, name in enumerate(STATE_STREAM_NAMES):
            receive_age_ns = int(stats.stream_receive_age_ns[index])
            valid_age_ns = int(stats.stream_valid_age_ns[index])
            streams[name] = {
                "messages": int(stats.stream_messages[index]),
                "invalid_messages": int(stats.stream_invalid_messages[index]),
                "receive_age_ms": (
                    receive_age_ns / 1e6 if receive_age_ns >= 0 else -1.0
                ),
                "valid_age_ms": (
                    valid_age_ns / 1e6 if valid_age_ns >= 0 else -1.0
                ),
                "source_timestamp_ns": int(
                    stats.stream_source_timestamp_ns[index]
                ),
            }
        source_skew_ns = int(stats.latest_source_skew_ns)
        return {
            "state_messages": int(stats.state_messages),
            "invalid_state_messages": int(stats.invalid_state_messages),
            "synchronized_frames": int(stats.synchronized_frames),
            "command_batches": int(stats.command_batches),
            "latest_source_skew_ms": (
                source_skew_ns / 1e6 if source_skew_ns >= 0 else -1.0
            ),
            "streams": streams,
        }

    def _inject_hardware_state_for_test(
        self, sequence: int, q, dq=None, tau_est=None, *,
        torso_quaternion_wxyz=None, torso_gyro=None, torso_acceleration=None,
        has_torso_imu: bool | None = None, pelvis_acceleration=None,
    ) -> None:
        """Test/transport-conformance hook; production perception stays C++."""
        state = _HardwareState()
        state.sequence = sequence
        state.timestamp_ns = sequence * 20_000_000
        state.sync_complete = 1
        state.pelvis_quaternion_wxyz[0] = 1.0
        state.pelvis_acceleration[2] = 9.81
        state.torso_quaternion_wxyz[0] = 1.0
        state.torso_acceleration[2] = 9.81
        _copy(q, state.q)
        _copy(np.zeros(HARDWARE_DOF) if dq is None else dq, state.dq)
        _copy(np.zeros(HARDWARE_DOF) if tau_est is None else tau_est, state.tau_est)
        if pelvis_acceleration is not None:
            _copy(pelvis_acceleration, state.pelvis_acceleration)
        if torso_quaternion_wxyz is not None:
            _copy(torso_quaternion_wxyz, state.torso_quaternion_wxyz)
        if torso_gyro is not None:
            _copy(torso_gyro, state.torso_gyro)
        if torso_acceleration is not None:
            _copy(torso_acceleration, state.torso_acceleration)
        if has_torso_imu is None:
            has_torso_imu = any(
                value is not None
                for value in (torso_quaternion_wxyz, torso_gyro, torso_acceleration)
            )
        state.has_torso_imu = int(bool(has_torso_imu))
        if not self._library.agi3sdk_publish_hardware_state(
            self._handle, ctypes.byref(state)
        ):
            raise SdkError("hardware state injection rejected")

    def _install_memory_command_writer_for_test(self) -> None:
        """Install an explicit no-hardware sink; never used by production."""
        if self._started:
            raise SdkError("install the command writer before start")
        self._test_writer = _CommandWriter(lambda _command, _user_data: 1)
        if not self._library.agi3sdk_set_command_writer(
            self._handle, self._test_writer, None
        ):
            raise SdkError("failed to install test command writer")

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if not handle:
            return
        self._library.agi3sdk_stop(handle)
        self._library.agi3sdk_destroy(handle)
        self._handle = None
        self._started = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.close()

    def __del__(self):
        self.close()
