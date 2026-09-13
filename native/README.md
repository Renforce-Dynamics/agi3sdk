# agi3sdk

`agi3sdk` 是 A3 真机 transport 与策略 runtime 之间的 C++ 边界。它不包含 FSM、ONNX、
planet-rally、Nokov 或 MuJoCo 逻辑，也不读取 `.reference/`。

## 已实现的接口

```text
6 路 A3 callback
  -> StateAssembler (完整性、timestamp skew、31-DOF native layout)
  -> RobotIO.PublishState()
  -> immutable PolicyState (sequence + timestamp + 29-DOF + pelvis/torso IMU)
  -> C ABI
  -> agi3sdk.A3RobotIO.read_state()

Python JointCommand(29)
  -> C ABI + state_sequence 校验
  -> 29→31 scatter，neck hold
  -> command writer
  -> 4 路 /body_drive command publisher
```

SDK 已实现：

- 六路样本 assembler 和最大同步偏差 gate；
- A3 native 31 维与策略 29 维的显式转换；
- immutable sequence snapshot、等待新帧、陈旧/乱序状态拒绝；命令绑定到实际交付给
  Python 的快照，不会被推理期间到达的高频 AimRT 新帧误判为 sequence mismatch；
- command finite/Kp/Kd/sequence/state-age 校验；
- SDK 内 C++ command watchdog，超时只发送一次全关节 damping；该 worker 与 Python
  ctypes runtime 位于同一进程，不能覆盖进程崩溃或 `SIGKILL`；
- C++ API、稳定的 C ABI 和不依赖 pybind 的 Python ctypes facade；
- 内嵌 AimRT 1.6 Core、Iceoryx backend、六路真实 subscriber 和四路真实 publisher；
- mock、C ABI、mapping、assembler、watchdog 测试；本轮独立仓库验证未连接真机。

这里不使用 ROS2 transport。厂商消息采用 ROS2-shaped C++ type support，但订阅/发布 API
是 AimRT，backend 只启用 Iceoryx，不创建 ROS2 node，也不启用 AimRT ROS2 plugin。

## 感知 callback 的调用方式

真机 adapter 分别订阅：

```text
/body_drive/waist_joint_state  -> UpdateWaist(JointGroupSample<3>)
/body_drive/neck_joint_state   -> UpdateNeck(JointGroupSample<2>)
/body_drive/arm_joint_state    -> UpdateArms(JointGroupSample<14>)
/body_drive/leg_joint_state    -> UpdateLegs(JointGroupSample<12>)
/body_drive/pelvis_imu/data    -> UpdatePelvisImu(ImuSample)
/body_drive/torso_imu/data     -> UpdateTorsoImu(ImuSample)
```

每次 callback 更新后调用 `TryAssemble()`；只有六路齐全、每一路都比上一帧有更新且
时间戳偏差不超过配置门限时才返回 `RobotState`。adapter 随后调用
`RobotIO::PublishState(state)`。
Python 每周期只调用 `read_state()`，不会看到任何半帧，也不接触 AimRT 类型。
`quaternion_wxyz/gyro_b` 仍保持为策略历史兼容用的 pelvis IMU；`torso_*` 字段单独透传
给上肢/腰部传统控制层，用于实机上身直立补偿。

硬件 31 维顺序是 `waist(3), neck(2), arms(14), legs(12)`；策略 29 维顺序是
`legs(12), waist(3), arms(14)`。映射定义在 `types.hpp` 并由单测锁定。

## 本机构建和测试

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --target agi3sdk agi3sdk_test
ctest --test-dir build --output-on-failure

./scripts/test.sh
```

Python facade 会自动查找 `build/native/libagi3sdk.so`，也可显式设置：

```bash
export AGI3SDK_LIBRARY=/absolute/path/to/libagi3sdk.so
```

在 SDK 仓库根目录使用 `scripts/build-native.sh -DAGI3SDK_WITH_AIMRT=ON -DAGI3_AIMRT_SOURCE_DIR=/path/to/AimRT` 原生构建。topic 名由应用配置经 Python/C ABI 传入 SDK。

正常 `Stop()` 和 command timeout 都会尝试发送一次 safe damping。该 watchdog 与 Python
位于同一进程，不能覆盖崩溃或 `SIGKILL`；真机 LOCO 前仍须确认进程外/硬件 command TTL，
集成入口见 SDK 根目录 `README.md` 及 cadence-rally 应用文档。
