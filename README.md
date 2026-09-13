# agi3sdk

**A3 robot I/O for C++ and Python.**

agi3sdk provides synchronized robot state, validated command submission and transport lifecycle management for A3. The native library exposes a C ABI and a Python `A3RobotIO` interface.

- Assemble complete state snapshots with sequence and timing checks.
- Validate commands and enforce command timeouts in the native transport.
- Develop against the mock transport; select AimRT for hardware deployment.

The SDK owns device I/O. Applications own control policies, task state machines and device activation.

## Quick start

Requires Linux, Python 3.10+ and `uv`. Native builds also require CMake and a C++ compiler.

```bash
git clone --recurse-submodules git@github.com:Renforce-Dynamics/agi3sdk.git
cd agi3sdk
./scripts/bootstrap.sh
./scripts/doctor.sh
./scripts/test.sh
cmake --install build --prefix "$PWD/install"
```

## Integration and configuration

There are no internal source dependencies. NumPy is installed as a Python dependency; CMake and a C++ compiler are required for native builds.

Transport options are supplied explicitly through the SDK API. Native build options select transport capabilities:

```bash
./scripts/build-native.sh -DAGI3SDK_WITH_AIMRT=ON -DAGI3_AIMRT_SOURCE_DIR=/path/to/AimRT
```

Hardware builds require the target ROS 2 message environment and AimRT sources. The Python wheel contains bindings; install the native library separately and set `AGI3SDK_LIBRARY` when it is outside the default search locations.

C++ consumers use `find_package(agi3sdk 0.1 CONFIG REQUIRED)` and link `agi3sdk::agi3sdk`; set `CMAKE_PREFIX_PATH` to the installation prefix.

See the [native API and transport guide](native/README.md) and [Python binding examples](tests/test_binding.py).

## Development

```bash
./scripts/submodules.sh init    # initialize or restore pinned dependencies
./scripts/submodules.sh check
./scripts/test.sh
./scripts/build.sh
```

Submodules pin source commits; Python requirements describe package compatibility. Bootstrap installs only the explicit packages in `source-workspace.json`. `scripts/setup.sh --wheelhouse /path/to/wheels` is available for package-based installation. Upgrade dependencies by committing reviewed submodule revisions with the parent repository.

## Authorship and license

Developed and maintained by [Renforce Dynamics](https://github.com/Renforce-Dynamics). See [AUTHORS.md](AUTHORS.md). Project code is available under the [MIT License](LICENSE).
