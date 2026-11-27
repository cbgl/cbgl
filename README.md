<h1 align='center' style="text-align:center; font-weight:bold; font-size:2.0em;letter-spacing:2.0px;"> CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor </h1>

<div align="center">

[![ieeexplore.ieee.org](https://img.shields.io/badge/IEEE/RSJ_IROS_2024_paper-00629B)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10802235) [![github.com](https://img.shields.io/badge/pitch-333333)](https://mozilla.github.io/pdf.js/web/viewer.html?file=https://raw.githubusercontent.com/cbgl/IROS-2024-presentations-resources/master/cbgl_pitch/presentation/IROS24_0101_presentation.pdf) [![youtube.com](https://img.shields.io/badge/2'_presentation-YouTube-FF0000)](https://www.youtube.com/watch?v=xaDKjI0WkDc) [![youtube.com](https://img.shields.io/badge/In_depth-YouTube-FF0000)](https://www.youtube.com/watch?v=TvTNEDGp-NU) [![build.ros2.org](https://build.ros2.org/job/Ldev__cbgl__ubuntu_resolute_amd64/badge/icon?subject=lyrical-devel)](https://build.ros2.org/job/Ldev__cbgl__ubuntu_resolute_amd64/)

</div>

`cbgl` localises a 2D lidar sensor in a known 2D occupancy grid from a single scan, in one shot. It works under global uncertainty in both position and orientation. It needs no motion and no initial guess. Give it a map and one scan, call it once, and it tells you where the sensor was.

This is the ROS 2 version. The ROS 1 Kinetic package is on the `kinetic-devel` branch. [Upgrading from the ROS 1 version](doc/upgrading-from-ros1.md) lists every name and every behaviour that changed.

https://github.com/li9i/cbgl/assets/1658819/5794cd21-651d-4924-b453-25c46b9e42a9

<sub>A panoramic 2D LIDAR sensor mounted on a turtlebot 2 is spawned into an environment at a pose whose immediate surroundings are repeated in (almost) the same geometry and proportions at locations other than the sensor's spawning ground. The user calls the global localisation service once before moving the robot at a second challenging pose, at which she calls it for a second time. Both times `cbgl` is successful in estimating the sensor's pose. Sensor characteristics: 360 rays; noise distributed normally about zero with a standard deviation of 0.05 m.</sub>

Table of Contents
=================

* [Requirements](#requirements)
* [Installation](#installation)
* [Run](#run)
* [Input and output](#input-and-output)
* [Parameters](#parameters)
* [Troubleshooting](#troubleshooting)
* [Performance](#performance)
* [Licensing](#licensing)
* [Citation](#citation)
* [Further documentation](#further-documentation)

## Requirements

* ROS 2 Lyrical
* A compiler with C++23
* [`fsm_localisation`](https://github.com/fourier-scan-matcher/fsm-localisation), which provides the scan matcher that refines each survivor, the ray casting that reads the map, and the occupancy grid type both use. `cbgl` carries no copy of any of them.
* A 2D lidar. CBGL is built for lidars with a field of view of 360 degrees, and lidars with a narrower field of view are supported too. The scan must state what it covers through `angle_min` and `angle_increment`, and it must be centred on the sensor's heading. A scan that is not centred is refused at the point of use. [Sensors that do not see all the way round](doc/parameters.md#sensors-that-do-not-see-all-the-way-round) gives the detail.

## Installation

The ROS package is named `cbgl_global_localisation`. The repository, the node and its executable `cbgl_interface_node` keep the name `cbgl`.

### With Docker

The image carries both this package and `fsm_localisation`, so the build context spans the directory that holds both repositories, not this one alone.

```sh
git clone git@github.com:cbgl/cbgl.git
git clone git@github.com:fourier-scan-matcher/fsm-localisation.git
cd cbgl/docker
docker compose build
```

By default the image launches the node, which puts it in stand-by. For a shell instead, run this:

```sh
docker compose run --rm cbgl bash
```

### From source

`fsm_localisation` must be in the same workspace. Everything else is declared in the manifests, and `rosdep` fetches it.

```sh
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:cbgl/cbgl.git
git clone git@github.com:fourier-scan-matcher/fsm-localisation.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to cbgl_global_localisation
```

[Building](doc/building.md) describes two build options for comparing builds, and how to link the libraries of this package from your own package.

## Run

### Launch

```sh
ros2 launch cbgl_global_localisation cbgl.launch.xml
```

The launch file takes three arguments. `namespace` defaults to `/`, `node_name` defaults to `cbgl`, and `params` defaults to the shipped `config/params.yaml`. Two nodes cannot share a name. To run a second node, launch this same file again with a name and a parameter file of its own.

Launching puts the node in **stand-by**. It holds the newest map and the newest scan, and it computes nothing until its service is called.

### Call

```sh
ros2 service call /cbgl/global_localisation std_srvs/srv/Trigger
```

The call returns when the localisation is **done**, which can be seconds later. The response says whether a pose was produced. Its message says how long the run took and what it searched, or why the run was refused. A second call that arrives while a run is under way is refused, not queued.

The service is node relative, so its name follows `node_name`.

### Taking the map from a file instead of a topic

Set `map_topic` to empty and point `map_file` at a map description file on disk. The node then subscribes to no map topic.

```yaml
/**:
  ros__parameters:
    map_topic: ""
    map_file: "/maps/room.yaml"
```

The description file has the format that a map server reads:

```yaml
image: room.pgm
resolution: 0.05
origin: [-1.35, 2.70, 0.60]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

`image`, `resolution` and `origin` are required. The image must be a PGM, either binary `P5` or plain `P2`, which is what ROS map saving writes. The node reads the file once, at start-up, and a file that it cannot use stops the node from starting. [Map files](doc/map-file.md) gives the optional keys, how the image is read and every reason for a refusal.

### Feeding it a map from a recording

The map subscription asks for **latched** delivery, so that a map published before the node started still arrives. A map server publishes that way. A plain recording playback does not, and a subscription that asks for latched delivery does not match a publisher that does not offer it. The node then waits, and every call is refused because no map has arrived.

To replay a recorded map, set the durability on the way out:

```sh
cat > map_qos.yaml <<'EOF'
/map:
  durability: transient_local
  reliability: reliable
  history: keep_last
  depth: 1
EOF
ros2 bag play <recording> --qos-profile-overrides-path map_qos.yaml
```

Scans need no such setting, because the scan subscription is ordinary.

## Input and output

| Direction | Topic | Type | What |
| --- | --- | --- | --- |
| in | `scan_topic` | `sensor_msgs/msg/LaserScan` | the measurement. The newest is held and nothing happens until the service is called |
| in | `map_topic` | `nav_msgs/msg/OccupancyGrid` | the map to localise in. Subscribed transient local at depth one, so a map latched before the node started arrives on subscription. Empty subscribes nothing and the map is read from `map_file` instead |
| out | `output_pose_topic` | `geometry_msgs/msg/PoseWithCovarianceStamped` | the answer, as a pose of the **robot**, in `fixed_frame_id`, stamped from the scan it was computed from |
| out | `hypotheses_topic` | `geometry_msgs/msg/PoseArray` | optional. Every hypothesis the search started from |
| out | `top_hypotheses_topic` | `geometry_msgs/msg/PoseArray` | optional. The survivors of the sift, in ranking order |
| out | `map_scan_topic` | `sensor_msgs/msg/LaserScan` | optional. The scan the answer would see, cast from the map, for laying over the scan the sensor did see |
| out | transform | `fixed_frame_id` <- `odom_frame_id` | only when `tf_broadcast` |

One service, `global_localisation`, of type `std_srvs/srv/Trigger`.

## Parameters

Every setting below lives in one file, `config/params.yaml`, keyed by the wildcard `/**:` so that renaming the node cannot silently drop them all. Each setting is **read once, at start-up, and is read only after that**. A runtime set is refused, not accepted and quietly ignored.

A value outside its permitted range is refused at start-up. The node names the setting and the value it was given, and exits with 1.

The **Default** column is what the node declares. The **Shipped** column is what `config/params.yaml` sets, which differs where reproducing the behaviour of the ROS 1 package asks for it. A table with no **Shipped** column ships every setting at its default. [Parameters in detail](doc/parameters.md) gives the reasons behind several settings and their history.

### Topics

| Parameter | Meaning | Default |
| --- | --- | --- |
| `scan_topic` | where 2D scans are published | `/scan` |
| `output_pose_topic` | where the answer is published | `/initialpose` |
| `hypotheses_topic` | optional. Every hypothesis drawn. Empty publishes nothing | empty |
| `top_hypotheses_topic` | optional. The survivors of the sift. Empty publishes nothing | empty |
| `map_scan_topic` | optional. The scan the answer would see. Empty publishes nothing | empty |

### Where the map comes from

One of the two settings below must name something. If both are empty, the node refuses to start. If both are set, the node uses the topic and warns that it ignored the file.

| Parameter | Meaning | Default |
| --- | --- | --- |
| `map_topic` | the topic the map arrives on. Empty subscribes nothing | `/map` |
| `map_file` | a map description file on disk, read once at start-up. Empty reads no file | empty |

### Frames

Frame names carry no leading slash. tf2 rejects one, and the node refuses one at start-up.

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `fixed_frame_id` | the frame the map and the published answer are expressed in | `map` | `map` |
| `base_frame_id` | the robot's own frame, which the published pose places | `base_footprint` | `base_footprint` |
| `odom_frame_id` | the odometry frame, read only when broadcasting | `odom` | `odom` |
| `tf_broadcast` | broadcast `fixed_frame_id` <- `odom_frame_id` for each answer | `false` | `true` |
| `tf_lookup_timeout` | seconds to wait for a transform before refusing a run | `0.1` | `0.1` |

The sensor's own frame is **not** a parameter. The node reads it from the scan message, and it finds where the sensor sits on the robot by looking `base_frame_id` up against it. That transform must exist in the tree, or every run is refused.

### Map

| Parameter | Meaning | Default |
| --- | --- | --- |
| `occupancy_threshold` | a cell at or above this, on the nought to one hundred scale, is blocked | `65` |
| `map_scan_method` | which algorithm casts a ray: `ray_marching`, `bresenham` or `cddt` | `ray_marching` |

A cell that no sensor has ever seen blocks a ray, whatever the value of `occupancy_threshold`. Every hypothesis is drawn on a free cell, so this setting also decides where the search may look.

### Dispersal and sift

| Parameter | Meaning | Default |
| --- | --- | --- |
| `dl` | hypothesis positions drawn per square metre of free space | `15.0` |
| `da` | candidate orientations scored per position | `32` |
| `top_k_caers` | how many of the scored candidates are refined | `10` |
| `rng_seed` | `0` draws from hardware entropy, any other value pins the run | `0` |
| `num_threads` | how many threads the sift is split over, `0` meaning what the hardware offers | `0` |

`dl` is a decimal and must be written as one, for example `15.0` and not `15`. A fractional density is valid.

`rng_seed` pins both the dispersal draw and the recovery search of the matcher, so a pinned run repeats exactly. `num_threads` changes neither the ranking nor the answer.

### Refinement

This setting chooses whether the survivors of the sift are refined against the map before the best of them is published.

| Parameter | Meaning | Default |
| --- | --- | --- |
| `refinement_method` | whether a survivor is refined: `fsm` or `none` | `fsm` |

`fsm` is the panoramic scan matcher of `fsm_localisation`. It reads the map afresh at every pose it tries, at whatever aperture the sensor covers. `none` publishes the best candidate of the sift as it is. That candidate is quantised to what the sift can express, so `none` is the cheapest and least accurate configuration. Any other value is refused at start-up. The ROS 1 package also offered Andrea Censi's canonical scan matcher, and this version does not carry it. [Refinement](doc/parameters.md#refinement) gives the reason and the measured figures.

### Matcher

These settings are handed through to `fsm_localisation`, and the validator of that package refuses a bad value.

| Parameter | Default | Shipped |
| --- | --- | --- |
| `size_scan` | `0` | `0` |
| `min_magnification_size` | `0` | `0` |
| `max_magnification_size` | `3` | `3` |
| `num_iterations` | `2` | `3` |
| `xy_bound` | `0.2` | `1.2` |
| `t_bound` | pi/4 | `0.786` |
| `max_counter` | `200` | `200` |
| `max_recoveries` | `10` | `10` |
| `ray_search` | `angular` | `angular` |
| `enforce_early_gearup` | `false` | `true` |
| `enforce_translation_bound` | `false` | `true` |
| `recover_on_failed_sift` | `false` | `true` |

`size_scan` is how many rays a scan is reduced to, once, before the sift. The shipped `0` uses every ray the scan carries, so it refuses no scan for its ray count. Any other value refuses a scan that carries fewer rays than that value. [Matcher](doc/parameters.md#matcher) explains `size_scan` in full and why some shipped values differ from the defaults.

### Covariance

| Parameter | Default |
| --- | --- |
| `initial_cov_xx` | `0.05` |
| `initial_cov_yy` | `0.05` |
| `initial_cov_aa` | `0.06854` |

These values are **not measured**, and they are **not an estimate of the uncertainty of this package**. They are what a consumer such as amcl is seeded with, and the node publishes them exactly as they are set here.

### Node behaviour

| Parameter | Meaning | Default |
| --- | --- | --- |
| `scan_qos_reliability` | `reliable` or `best_effort` | `reliable` |
| `scan_qos_depth` | subscription queue depth | `1` |

`reliable` mirrors what ROS 1 did. Sensor drivers commonly publish `best_effort`, and a subscription that does not match its publisher receives nothing at all. So `scan_qos_reliability` is the first setting to check when a scan never seems to arrive.

## Troubleshooting

A refused run publishes no pose. The service response gives the reason as a sentence. The table below lists the most common reasons. [Why a run is refused](doc/troubleshooting.md) lists all of them.

| Reason in the response | What to check |
| --- | --- |
| no map has arrived | `map_topic` or `map_file`. For a recording, the durability in [Feeding it a map from a recording](#feeding-it-a-map-from-a-recording) |
| no scan has arrived | `scan_topic`, and `scan_qos_reliability` against what the sensor driver publishes |
| the sensor's placement is not in the transform tree | the transform between `base_frame_id` and the frame of the scan |
| a run is already under way | the earlier call, which must return first |
| the scan is not centred on the sensor's heading | `angle_min` and `angle_increment` of the scan. See [The scan is not centred](doc/troubleshooting.md#the-scan-is-not-centred) |
| the scan is shorter than `size_scan` | `size_scan`. The shipped `0` refuses no scan for its ray count |

## Performance

`size_scan` and `top_k_caers` are the two settings to lower when a run takes too long. In the room where the timings were measured, lowering `size_scan` saved more, most of all on the first call after a map arrives. [Performance](doc/performance.md) gives the measured execution times, and explains what the map costs and what it bounds.

## Licensing

The sources of `cbgl` are MIT. See [LICENSE](LICENSE). A redistributed binary carries further obligations through `fsm_localisation`, among them GPL v3 through CGAL. The GPL v3 text is in [COPYING](COPYING) and is installed with the package. [Licensing](doc/licensing.md) gives the full reading. It is not legal advice.

## Citation

The article below, from the 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems, presents the theory, the key aspects and the experimental performance of CBGL. [Background](doc/background.md) holds three figures on the motivation for CBGL and on how it works.

```bibtex
@INPROCEEDINGS{10802235,
  author={Filotheou, Alexandros},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor},
  year={2024},
  pages={3268-3275},
  doi={10.1109/IROS58592.2024.10802235}}
```

## Further documentation

* [Building](doc/building.md) describes the build options for comparing builds, and how to use this package as a dependency of your own package.
* [Parameters in detail](doc/parameters.md) gives the reasons behind several settings, and the history of the refinement and of `size_scan`.
* [Map files](doc/map-file.md) describes the map description file and how the node reads its image.
* [Why a run is refused](doc/troubleshooting.md) lists every reason for a refused run.
* [Performance](doc/performance.md) describes what the map costs and bounds, and what governs execution time.
* [Upgrading from the ROS 1 version](doc/upgrading-from-ros1.md) lists every renamed setting, every removed setting and every changed behaviour.
* [Background](doc/background.md) holds three figures on the motivation for CBGL and on how it works.
* [Licensing](doc/licensing.md) gives the licence of the sources and the obligations of a redistributed binary.
* [FOLLOWUP.md](FOLLOWUP.md) records known gaps and deferred work.
