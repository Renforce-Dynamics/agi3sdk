# agi3sdk

独立的 A3 硬件接口：C++ 负责状态拼接、命令生命周期、超时和传输，Python 提供 `A3RobotIO`、状态与命令类型。SDK 不依赖 Cadence，不包含策略、手柄、planner 或乒乓球状态机。

```bash
./scripts/setup.sh
./scripts/build-native.sh
./scripts/test.sh
./scripts/build.sh                       # Python 绑定 wheel
cmake --install build --prefix "$PWD/install"  # C++ 库与头文件
```

默认构建 mock 传输，可在没有机器人的机器上验证。Python 优先使用显式库路径、`AGI3SDK_LIBRARY`、当前 SDK 的 build 或已安装的库。分发到另一台机器时安装 CMake 产物并设置 `AGI3SDK_LIBRARY=/path/to/libagi3sdk.so`；Python wheel 不伪装成包含本地二进制的跨平台包。

真机 transport 构建：

```bash
./scripts/build-native.sh -DAGI3SDK_WITH_AIMRT=ON -DAGI3_AIMRT_SOURCE_DIR=/path/to/AimRT
```

需先加载目标机器的 ROS 2 / joint_msgs 环境。绑定会检查传输是否编译启用；实际机器人通讯、时序和负载须在该环境验证。

接口与 native 使用方法见 [native/README.md](native/README.md)；Python 示例见 `tests/test_binding.py`。本仓库只由业务应用显式打开设备。

C++ 消费者使用安装后的 CMake package：

```cmake
find_package(agi3sdk 0.1 CONFIG REQUIRED)
target_link_libraries(my_controller PRIVATE agi3sdk::agi3sdk)
```

配置消费项目时通过 `CMAKE_PREFIX_PATH` 指定上述 install 目录。
