# Building

This page describes two build options for comparing builds, and how to use this package as a dependency of your own package. [Installation](../README.md#installation) in the README gives the normal build.

## Build options for comparing builds

Two build options exist for comparing builds, not for running a robot. Both cost speed and both default to off. **Both belong to `fsm_localisation`, not to this package.** This package follows whatever `fsm_localisation` was built with.

| Option | What it does | What it is for |
| --- | --- | --- |
| `FSM_LOCALISATION_EXACT_FP` | builds at `-O2 -fno-fast-math -ffp-contract=off` instead of `-Ofast` | two compilers agree |
| `FSM_LOCALISATION_DETERMINISTIC_FFT` | the transform plans of the matcher are chosen by heuristic, not by timing | two runs agree |

### From source

```sh
colcon build --packages-up-to cbgl_global_localisation --cmake-args \
  -DFSM_LOCALISATION_EXACT_FP=ON -DFSM_LOCALISATION_DETERMINISTIC_FFT=ON
```

### With Docker

The image runs a build, so the two options reach it as build arguments, not as CMake arguments. Both options belong to `fsm_localisation`, and this package follows whatever that package was built with, so there are two arguments and not four. Run this from the `docker` directory of this repository:

```sh
docker compose build \
  --build-arg FSM_LOCALISATION_EXACT_FP=ON \
  --build-arg FSM_LOCALISATION_DETERMINISTIC_FFT=ON
```

## As a dependency of your own package

Five libraries are exported. Link the narrowest one that does what you need.

| Target | Knows about | Use it for |
| --- | --- | --- |
| `cbgl_global_localisation::cbgl_core` | nothing of ROS, not even a message type | the algorithm itself, given a grid and a scan |
| `cbgl_global_localisation::cbgl_grid` | occupancy grid messages, no `rclcpp` | turning a map message into the free space the search draws from |
| `cbgl_global_localisation::cbgl_map_file` | occupancy grid messages and `yaml-cpp`, no `rclcpp` | reading a map description file and its PGM into a map message |
| `cbgl_global_localisation::cbgl_interface` | everything ROS | embedding the node in a process of your own |

Every target above reaches CGAL and FFTW through `fsm_localisation`. See [Licensing](licensing.md).

```cmake
find_package(cbgl_global_localisation REQUIRED)
target_link_libraries(your_target cbgl_global_localisation::cbgl_core)
```
