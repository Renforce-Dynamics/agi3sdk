# agi3sdk

A3 机器人 I/O：AimRT 状态接收、关节命令提交、同步与超时检查，提供 C++、C ABI 和 Python 接口。

## 安装

Linux、Python 3.10+、`uv`、CMake 和 C++ 编译器。

```bash
git clone git@github.com:Renforce-Dynamics/agi3sdk.git
cd agi3sdk
./scripts/bootstrap.sh
```

## 构建真机接口

先按 [A3 环境准备](https://github.com/Renforce-Dynamics/cadence/blob/main/docs/a3-onboard.md) 加载厂商环境、ROS 消息依赖并准备 AimRT 源码，再构建：

```bash
./scripts/build-native.sh \
  -DAGI3SDK_WITH_AIMRT=ON \
  -DAGI3_AIMRT_SOURCE_DIR=/path/to/AimRT

export AGI3SDK_LIBRARY="$PWD/build/native/libagi3sdk.so"
cmake --install build --prefix "$PWD/install"
```

`bootstrap.sh` 默认构建开发用 mock transport；真机使用上面的 AimRT 构建。Python wheel 提供绑定，native 库单独安装。

## 接入

Python 使用 `agi3sdk.A3RobotIO`：`start()` → `read_state()` → `write_command()` → `close()`；`read_hardware_state()`（C ABI 6，`agi3sdk_copy_last_hardware_state`）只读返回最新组装的 31 维硬件状态（body-drive 顺序 waist 3、neck 2、arms 14、legs 12，含实测颈部角），ABI 5 的库仍可加载但没有这个函数；参数与调用示例见 [绑定测试](tests/test_binding.py) 和 [原生接口](native/README.md)。

C++ 使用安装目录中的 CMake package：

```cmake
find_package(agi3sdk 0.1 CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE agi3sdk::agi3sdk)
```

SDK 负责设备 I/O；HAL、PM 与应用启动顺序见 [A3 操作教程](https://github.com/Renforce-Dynamics/cadence/blob/main/docs/a3-onboard.md)。

## 开发

```bash
./scripts/doctor.sh
./scripts/test.sh
./scripts/build.sh
```

Renforce Dynamics · [作者](AUTHORS.md) · [MIT](LICENSE)
