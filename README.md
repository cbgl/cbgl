<h1 align='center' style="text-align:center; font-weight:bold; font-size:2.0em;letter-spacing:2.0px;"> CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor </h1>

<div align="center">

[![ieeexplore.ieee.org](https://img.shields.io/badge/IEEE/RSJ_IROS_2024_paper-00629B)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10802235) [![github.com](https://img.shields.io/badge/pitch-333333)](https://mozilla.github.io/pdf.js/web/viewer.html?file=https://raw.githubusercontent.com/cbgl/IROS-2024-presentations-resources/master/cbgl_pitch/presentation/IROS24_0101_presentation.pdf) [![youtube.com](https://img.shields.io/badge/2'_presentation-YouTube-FF0000)](https://www.youtube.com/watch?v=xaDKjI0WkDc) [![youtube.com](https://img.shields.io/badge/In_depth-YouTube-FF0000)](https://www.youtube.com/watch?v=TvTNEDGp-NU)

</div>

`cbgl` localises a 2D lidar sensor in a known 2D occupancy grid under global uncertainty in both position and orientation, from a single scan, in one shot. It needs no motion and no initial guess: give it a map, give it one scan, ask it once, and it tells you where the sensor was.

This is the ROS 2 version. The published ROS 1 Kinetic package is preserved unchanged on the `kinetic-devel` branch, and [Upgrading from the ROS 1 version](#upgrading-from-the-ros-1-version) lists every name and every behaviour that changed.

https://github.com/li9i/cbgl/assets/1658819/5794cd21-651d-4924-b453-25c46b9e42a9

<sub>A panoramic 2D LIDAR sensor mounted on a turtlebot 2 is spawned into an environment at a pose whose immediate surroundings are repeated in (almost) the same geometry and proportions at locations other than the sensor's spawning ground. The user calls the global localisation service once before moving the robot at a second challenging pose, at which she calls it for a second time. Both times `cbgl` is successful in estimating the sensor's pose. Sensor characteristics: 360 rays; noise distributed normally about zero with a standard deviation of 0.05 m.</sub>

Table of Contents
=================

* [How it works, in four steps](#how-it-works-in-four-steps)
* [Requirements](#requirements)
* [Installation](#installation)
  * [With Docker](#with-docker)
  * [From source](#from-source)
  * [As a dependency of your own package](#as-a-dependency-of-your-own-package)
* [Run](#run)
  * [Launch](#launch)
  * [Call](#call)
  * [Feeding it a map from a recording](#feeding-it-a-map-from-a-recording)
* [Input and output](#input-and-output)
* [Parameters](#parameters)
  * [Topics](#topics)
  * [Frames](#frames)
  * [Map](#map)
  * [Dispersal and sift](#dispersal-and-sift)
  * [Matcher](#matcher)
  * [Covariance](#covariance)
  * [Node behaviour](#node-behaviour)
* [Why a run is refused](#why-a-run-is-refused)
* [What the map costs and what it bounds](#what-the-map-costs-and-what-it-bounds)
* [What governs execution time](#what-governs-execution-time)
* [Upgrading from the ROS 1 version](#upgrading-from-the-ros-1-version)
* [Licensing](#licensing)
* [Motivation and under the hood](#motivation-and-under-the-hood)

## How it works, in four steps

Nothing in the ROS 2 version changes the shape of the algorithm. Every setting below governs one of these four steps.

1. **Disperse.** Draw hypothesis sensor poses over the map's free space: `dl` positions per square metre of it, each the centre of a free cell chosen uniformly at random, each carrying a uniformly drawn orientation.
2. **Sift.** For each drawn position, cast one panoramic scan from the map. Rotating that one range vector gives `da` orientations of the same position for the price of a single cast. Score every candidate against the measured scan by the cumulative absolute error per ray, and keep the `top_k_caers` cheapest.
3. **Refine.** Scan-match each survivor against the map, which yields a corrected pose per survivor.
4. **Select.** Score each corrected pose by the same metric and publish the best.

Steps 2 and 3 are where the time goes, and the second of the two dominates. See [What governs execution time](#what-governs-execution-time).

## Requirements

* ROS 2 Lyrical
* A compiler with C++23
* [`fsm_localisation`](https://github.com/fourier-scan-matcher/fsm-localisation), which provides the scan matcher that refines each survivor, the ray casting that reads the map, and the occupancy grid type both use. `cbgl` carries no copy of any of them.
* A 2D lidar that sees **all the way round**. A sensor with a narrower field of view is not supported. The ROS 1 version carried a second refinement path for that case, an ICP match, and this version does not carry it.

## Installation

### With Docker

The image carries both this package and `fsm_localisation`, so the build context spans the directory that holds both repositories rather than this one alone.

```sh
git clone git@github.com:cbgl/cbgl.git
git clone git@github.com:fourier-scan-matcher/fsm-localisation.git
cd cbgl/docker
docker compose build
```

The image runs a build, so the two arithmetic build options described below reach it as build arguments rather than as CMake arguments. Both packages are built in the one invocation, so all four arguments are set together for the same reason the source build sets all four:

```sh
docker compose build \
  --build-arg CBGL_EXACT_FP=ON --build-arg CBGL_DETERMINISTIC_FFT=ON \
  --build-arg FSM_LOCALISATION_EXACT_FP=ON \
  --build-arg FSM_LOCALISATION_DETERMINISTIC_FFT=ON
```

Launching puts the node in stand-by, which is what the image does by default. For a shell instead:

```sh
docker compose run --rm cbgl bash
```

### From source

`fsm_localisation` must be in the same workspace. Everything else is declared in the manifests and `rosdep` fetches it, bar the one stale rule described below.

```sh
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:cbgl/cbgl.git
git clone git@github.com:fourier-scan-matcher/fsm-localisation.git
cd ~/ros2_ws
sudo apt install libfftw3-dev
rosdep install --from-paths src --ignore-src -r -y --skip-keys libfftw3
colcon build --packages-up-to cbgl
```

`rosdep` needs `--skip-keys libfftw3`: the key resolves through the upstream rules to a package Ubuntu dropped in favour of one per floating point precision, so install `libfftw3-dev` by name instead.

Two build options exist for comparing builds rather than for running a robot. Both cost speed and both default off. `CBGL_EXACT_FP` builds at `-O2 -fno-fast-math -ffp-contract=off` instead of `-Ofast`, so that two compilers agree. `CBGL_DETERMINISTIC_FFT` makes the matcher's transform plans chosen by heuristic rather than by timing, so that two runs agree. Set them on **both** packages in the same invocation, not on `cbgl` alone:

```sh
colcon build --packages-up-to cbgl --cmake-args \
  -DCBGL_EXACT_FP=ON -DCBGL_DETERMINISTIC_FFT=ON \
  -DFSM_LOCALISATION_EXACT_FP=ON -DFSM_LOCALISATION_DETERMINISTIC_FFT=ON
```

### As a dependency of your own package

Three libraries are exported. Link the narrowest one that does what you need.

| Target | Knows about | Use it for |
| --- | --- | --- |
| `cbgl::cbgl_core` | nothing of ROS, not even a message type | the algorithm itself, given a grid and a scan |
| `cbgl::cbgl_grid` | occupancy grid messages, no `rclcpp` | turning a map message into the free space the search draws from |
| `cbgl::cbgl_interface` | everything ROS | embedding the node in a process of your own |

```cmake
find_package(cbgl REQUIRED)
target_link_libraries(your_target cbgl::cbgl_core)
```

## Run

### Launch

```sh
ros2 launch cbgl cbgl.launch.xml
```

The launch takes `namespace`, defaulting to `/`, `node_name`, defaulting to `cbgl`, and `params`, defaulting to the shipped `config/params.yaml`. Two nodes may not share a name, so a second node is this same file launched again with a name and a parameter file of its own.

Launching puts the node in **stand-by**. It holds the newest map and the newest scan and computes nothing at all until its service is called.

### Call

```sh
ros2 service call /cbgl/global_localisation std_srvs/srv/Trigger
```

The call returns when the localisation is **done**, which can be seconds later, and it carries the outcome: whether a pose was produced, and a sentence saying either how long it took and what it searched, or why it was refused. A second call arriving while one is under way is refused rather than queued.

The service is node relative, so it follows `node_name`.

### Feeding it a map from a recording

The map subscription asks for **latched** delivery, so that a map published before this node started still arrives. A map server publishes that way and needs nothing said. A plain recording playback does not, and a subscription asking for latched delivery does not match a publisher that does not offer it, so the node waits and every call is refused for want of a map with the recording running in front of it.

Replaying a recorded map therefore needs the durability set on the way out:

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

Scans need nothing of the sort, the scan subscription being ordinary.

## Input and output

| Direction | Topic | Type | What |
| --- | --- | --- | --- |
| in | `scan_topic` | `sensor_msgs/msg/LaserScan` | the measurement. The newest is held and nothing happens until the service is called |
| in | `map_topic` | `nav_msgs/msg/OccupancyGrid` | the map to localise in. Subscribed transient local at depth one, so a map latched before the node started arrives on subscription |
| out | `output_pose_topic` | `geometry_msgs/msg/PoseWithCovarianceStamped` | the answer, as a pose of the **robot**, in `global_frame_id`, stamped from the scan it was computed from |
| out | `hypotheses_topic` | `geometry_msgs/msg/PoseArray` | optional. Every hypothesis the search started from |
| out | `top_hypotheses_topic` | `geometry_msgs/msg/PoseArray` | optional. The survivors of the sift, in ranking order |
| out | `map_scan_topic` | `sensor_msgs/msg/LaserScan` | optional. The scan the answer would see, cast from the map, for laying over the scan the sensor did see |
| out | transform | `global_frame_id` <- `odom_frame_id` | only when `tf_broadcast` |

One service, `global_localisation`, of type `std_srvs/srv/Trigger`.

## Parameters

Every setting below lives in one file, `config/params.yaml`, keyed by the wildcard `/**:` so that renaming the node cannot silently drop the lot. Each is **read once, at start-up, and is read only thereafter**: setting one at runtime is refused rather than accepted and quietly ignored.

A value outside its permitted range is refused at start-up. The node says which setting and which value it was given, and exits with 1.

The **Default** column is what the node declares. The **Shipped** column is what `config/params.yaml` sets, which differs where reproducing the ROS 1 package's behaviour asks for it.

### Topics

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `scan_topic` | where 2D panoramic scans are published | `/scan` | `/scan` |
| `map_topic` | the map to localise in | `/map` | `/map` |
| `output_pose_topic` | where the answer is published | `/initialpose` | `/initialpose` |
| `hypotheses_topic` | optional. Every hypothesis drawn. Empty publishes nothing | empty | empty |
| `top_hypotheses_topic` | optional. The survivors of the sift. Empty publishes nothing | empty | empty |
| `map_scan_topic` | optional. The scan the answer would see. Empty publishes nothing | empty | empty |

### Frames

Frame names carry no leading slash. tf2 rejects one, and one is refused at start-up.

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `global_frame_id` | the frame the map and the published answer are expressed in | `map` | `map` |
| `robot_frame_id` | the robot's own frame, which the published pose places | `base_footprint` | `base_footprint` |
| `odom_frame_id` | the odometry frame, read only when broadcasting | `odom` | `odom` |
| `tf_broadcast` | broadcast `global_frame_id` <- `odom_frame_id` for each answer | `false` | `true` |
| `tf_lookup_timeout` | seconds to wait for a transform before refusing a run | `0.1` | `0.1` |

The sensor's own frame is **not** a parameter. It is read off the scan message, and the node works out where the sensor sits on the robot by looking `robot_frame_id` up against it. That transform must exist in the tree or every run is refused for want of it.

`tf_broadcast` defaults to false because broadcasting into somebody else's transform tree is intrusive. The shipped file turns it on, which is what the ROS 1 package shipped.

### Map

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `occupancy_threshold` | a cell at or above this, on the nought to one hundred scale, is blocked | `65` | `65` |
| `map_scan_method` | which algorithm casts a ray: `ray_marching`, `bresenham` or `cddt` | `ray_marching` | `ray_marching` |

A cell no sensor has ever seen blocks a ray whatever `occupancy_threshold` is set to. Every hypothesis is drawn on a free cell, so this setting decides where the search may look as well as what a ray stops at.

### Dispersal and sift

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `dl` | hypothesis positions drawn per square metre of free space | `15.0` | `15.0` |
| `da` | candidate orientations scored per position | `32` | `32` |
| `top_k_caers` | how many of the scored candidates are refined | `10` | `10` |
| `rng_seed` | `0` draws from hardware entropy, any other value pins the run | `0` | `0` |
| `num_threads` | how many threads the sift is split over, `0` meaning what the hardware offers | `0` | `0` |

`dl` is a decimal and must be written as one: `15.0` rather than `15`, a fractional density being legitimate.

`rng_seed` pins both the dispersal draw and the matcher's recovery search, so a pinned run repeats exactly. `num_threads` changes neither the ranking nor the answer: the same input gives the same answer at one thread and at all of them.

### Matcher

Handed through to `fsm_localisation` and refused by that package's own validator rather than by a second copy of it.

| Parameter | Default | Shipped |
| --- | --- | --- |
| `size_scan` | `0` | `360` |
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

`size_scan` is how many rays a scan is reduced to, once, before the sift. The same reduced scan is what the sift, the refinement and the selection all work from, and the cast from the map is taken at the same length. `0` uses every ray the scan carries, and a scan carrying fewer rays than this is refused.

`xy_bound` is shipped at 1.2 m against the matcher's own default of 0.2 m, because a survivor of the sift stands at the centre of a cell and can be a cell or more from the truth.

The last three settings are behaviours that are **unconditional** in the matcher this algorithm originally shipped with, and optional in the one it now borrows. The shipped file turns all three on, because leaving them off would be refining with a different matcher and calling it the same one.

`ray_search` defaults to `angular`, which did not exist when the ROS 1 package was written. `windowed` is what that package used, and it is kept so that a run can be compared against results published before `angular` existed.

### Covariance

| Parameter | Default | Shipped |
| --- | --- | --- |
| `initial_cov_xx` | `0.05` | `0.05` |
| `initial_cov_yy` | `0.05` | `0.05` |
| `initial_cov_aa` | `0.06854` | `0.06854` |

These are **not measured** and they are **not an estimate of this package's own uncertainty**. They are what a consumer such as amcl is seeded with, and they are published exactly as they are set here.

### Node behaviour

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `scan_qos_reliability` | `reliable` or `best_effort` | `reliable` | `reliable` |
| `scan_qos_depth` | subscription queue depth | `1` | `1` |

`reliable` mirrors what ROS 1 did. Sensor drivers commonly publish `best_effort`, and a subscription that does not match its publisher receives nothing at all, so this is the first setting to check when a scan never seems to arrive.

## Why a run is refused

No pose is published for a refused run. Each reason is returned as a sentence in the service response.

| Reason | Decided by |
| --- | --- |
| no map has arrived | the node |
| no scan has arrived | the node |
| the sensor's placement is not in the transform tree | the node |
| a run is already under way | the node |
| the scan is not panoramic | the node |
| the map has no free space at all | the algorithm |
| an empty hypothesis set was handed in | the algorithm |
| the scan is shorter than `size_scan` | the algorithm |
| the scan carries no usable range | the algorithm |
| every survivor was refused by the matcher | the algorithm, naming how many were refused for each reason |
| no corrected pose could be scored | the algorithm |
| the run did not finish, and what stopped it | anywhere. A run that runs out of memory on a large map is the case this exists for, and it answers with a sentence rather than falling silent |

"Not panoramic" needs a word. The map is cast over exactly one turn beginning half a turn behind the sensor's heading, and the measured scan is read the same way. A scan whose first ray sits anywhere else is a legal scan that this algorithm would read as though it began where it did not, and the answer would come back rotated. It is refused, naming the field and the value it carried, rather than answered wrongly.

## What the map costs and what it bounds

The map is the floor under the accuracy, not the algorithm. A cast ray stops at a cell, so what the search matches against is the room to the nearest cell rather than the room itself. That floor sits at a fraction of a cell rather than at a whole one: `fsm_localisation` measures one correction against a map at about a fifteenth of a cell, at five centimetre cells and at twenty centimetre cells alike. A coarser map raises the floor. It does not raise it to the cell size.

A finer map does not draw more hypotheses. `dl` is a density per square metre of free space, and free area is the number of free cells times the square of the cell size, so halving the cell size leaves the count where it was. What a finer map costs is the memory the grid takes and the once per map work of preparing it, both of which grow with the cell count.

The map also bounds where the search may look. Every hypothesis is drawn on a cell the map calls free, so a map whose free space is wrong in a corner cannot produce an answer in that corner, and a map whose free space is a small part of its extent costs no more to sample than a full one.

## What governs execution time

`top_k_caers` is the setting to lower when a run takes too long. Measured through the service, in the container, on one generated room of about five and a half square metres of free space. The first call after a map arrives pays once for work every later call reuses, so both are given:

| Configuration | Threads | Candidates scored | First call | Later calls |
| --- | --- | --- | --- | --- |
| the shipped settings | all | 2,656 | 0.35 s | 0.05 s |
| ten times the hypothesis density | one | 26,688 | 0.56 s | 0.21 s |
| ten times the density, fifty times the survivors | one | 26,688 | 5.98 s | 3.19 s |
| ten times the density, one hundred and fifty times the survivors | one | 26,688 | 9.46 s | 8.85 s |

The first row differs from the second in its thread count as well as in its density, so the three one thread rows are the ones to read against each other. Ten times as many candidates costs a fifth of a second. Fifty times as many survivors costs three seconds. The **refinement dominates**, not the sift, so the number of candidates a map's size implies matters much less than how many of them are refined.

These figures come from that one small room and from nothing larger. The number of hypotheses grows with the map's free area, so a large building at `dl: 15.0` draws hundreds of thousands of candidates. Whether the concurrency of the sift holds up at that scale is not measured here, and nothing in this readme should be read as saying it is.

## Upgrading from the ROS 1 version

The ROS 1 Kinetic package is on `kinetic-devel` and is unchanged. This version is not a drop-in replacement.

**Before you go back to that branch to check a behaviour, know that it cannot localise as it ships.** With the `map_scan_method` its own configuration file selects, the caster it casts every ray into is built by a constructor that is handed a bare pixel buffer carrying no dimensions and assigns neither the width nor the height it then bounds its loops by, so it cannot build a grid corresponding to the map. The other route into that constructor, from a file, is worse: it reads the dimensions correctly and then discards the object that would have held the grid. The alternative scanner the same parameter offers does not use that code at all and works, so the branch ships misconfigured rather than incapable, and the difference is one value in one file.

Two commits of 1 and 2 July 2024 put it there and it has had no working configured map path since. The commit before them, `fb9dab1` of 15 May 2024, is the last state that runs, and its own sources had stood unchanged for 208 days by then. Its caster is byte identical to the code the published experiments were run with. If you need the old behaviour for comparison, that is the commit to use.

A second fault is independent of the configuration: the map callback declares a stack array of four bytes per cell, so any building-sized map at a normal resolution overflows an ordinary stack before the node does anything at all.

**Renamed or moved**

| ROS 1 | ROS 2 |
| --- | --- |
| service `global_localization`, `std_srvs/Empty`, node relative under a configurable leaf | service `global_localisation`, `std_srvs/srv/Trigger`, node relative and fixed |
| node `cbgl_node` in namespace `robot`, so the service answered at `/robot/cbgl_node/global_localization` | node `cbgl` in namespace `/`, so it answers at `/cbgl/global_localisation`. Both are launch arguments |
| `fixed_frame_id` | `global_frame_id` |
| `base_frame_id`, which named the frame the published pose places | `robot_frame_id`, same meaning and same `base_footprint` default. **Rename the setting rather than dropping it**, or a robot whose own frame is not `base_footprint` will have every answer placed for the wrong frame |
| `configuration_files/params_cbgl.yaml`, `params_csm.yaml`, `params_fsm.yaml` | one `config/params.yaml`, keyed `/**:` |
| `launch/cbgl.launch` | `launch/cbgl.launch.xml` |

**Gone**

| Setting | Why |
| --- | --- |
| `do_icp`, `do_fsm` | one refinement path remains, the scan matcher, so there is nothing to choose. **The ROS 1 package shipped `do_icp: true`**, so anyone running its shipped configuration was refining by ICP, and that is the path that is gone. This is also why a sensor that does not see all the way round is no longer supported |
| every setting of `params_csm.yaml` | the ICP library is gone with the path that used it |
| `laser_z_orientation` | it existed to compensate for reading the grid's rows in reverse. The replacement map handles the origin's translation, its rotation and its row order itself |
| the `vanilla` value of `map_scan_method` | it selected a second ray tracer that is gone. The three surviving names are `ray_marching`, `bresenham` and `cddt`, and any other name is refused at start-up. `vanilla` was also what ROS 1 fell back to when the setting was absent, so a configuration that never named one has to name one now |
| `do_undersample_scan`, `undersample_rate` | `size_scan` does this, and does it by resampling rather than by dropping every nth ray |
| `publish_pose_sets` | replaced by three optional topics, each off while its name is empty |
| `global_localisation_service_name` | the service is node relative and fixed |
| `pkg_name` | the logger already provides a prefix |

**Behaviour that changed**

* **The service waits.** It returns when the work is done and carries the outcome. The ROS 1 service returned immediately and reported nothing about the run.
* **The refinement matches against the map.** The ROS 1 version's scan matching path matched each survivor against one scan manufactured at that survivor's own pose. This one casts into the map at every pose it tries. This is the largest behavioural difference in the port and the one most likely to move the accuracy figures.
* **`tf_broadcast: false` works.** In the ROS 1 version, turning broadcasting off meant the sensor's placement on the robot was never looked up, and every cast composed the robot pose with an uninitialised transform, so the published pose was arbitrary. Here the placement is read on every run whether broadcasting is on or off, which is why the node's own default for `tf_broadcast` can be false.
* **A rotated cast is labelled with the orientation the rotation actually amounts to.** The ROS 1 version shifted the range vector by one number of samples and labelled the result with a different angle, drifting to nearly eight degrees by the last candidate and in the opposite direction, so the refinement started from a pose that disagreed with the scan it was chosen for.
* **Hypotheses are drawn on free cells.** The ROS 1 version computed sampling bounds that swapped the map's centre for its half extent. For a map message origin at zero, which is where a map server usually puts it, the two arithmetics agree and the defect is latent. For an origin away from zero they diverge, and for a map centred on the world origin, which is what a mapping run started at the origin produces, the interval collapses to a single point the rejection loop spins on without returning. Dispersal by free cell removes the arithmetic entirely.
* **The poses drawn are sensor poses.** The ROS 1 version dispersed robot poses over free cells and offset to the sensor afterwards, which can place the sensor, which is what casts, on a blocked cell.
* **No pose is published when a run fails.** The ROS 1 version published one regardless, reading its winner from index minus one. It never refused a candidate, scoring every one unconditionally; the index stayed at minus one when the survivor list was empty, which `top_k_caers` at zero produces.
* **The published pose is stamped from the scan** that produced it, not from the wall clock, and its frame is `global_frame_id` rather than the hardcoded string `/map`.
* **A parameter outside its range refuses to start.** The node says which setting and which value, and exits with 1.

## Licensing

`cbgl` is MIT. See [LICENSE](LICENSE).

The ROS 1 version also carried LGPL v2.1 and Apache 2.0 obligations: the first from amcl's map and particle filter, the second from a vendored ray casting library. Both are gone from these sources along with the code that brought them.

A binary built from these sources links `fsm_localisation`, and therefore carries, if redistributed, the GPL v3 terms of the CGAL components inside that package and the Apache 2.0 terms of the ray casting inside it. Building and running for yourself is unaffected. Redistributing a binary is not.

## Motivation and under the hood

<p align="center">
  <img src="https://i.imgur.com/kAD8AmS.png?1">
</p>

<p align="center">
  <img src="https://i.imgur.com/LQBwg7G.png">
</p>

<p align="center">
  <img src="https://i.imgur.com/0qt3chL.png">
</p>

The theoretical underpinning, key aspects, and experimental performance of CBGL are presented in the 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems article cited through

```bibtex
@INPROCEEDINGS{10802235,
  author={Filotheou, Alexandros},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor},
  year={2024},
  pages={3268-3275},
  doi={10.1109/IROS58592.2024.10802235}}
```

Known gaps and deferred work are recorded in [FOLLOWUP.md](FOLLOWUP.md).
