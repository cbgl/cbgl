<h1 align='center' style="text-align:center; font-weight:bold; font-size:2.0em;letter-spacing:2.0px;"> CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor </h1>

<div align="center">

[![ieeexplore.ieee.org](https://img.shields.io/badge/IEEE/RSJ_IROS_2024_paper-00629B)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10802235) [![github.com](https://img.shields.io/badge/pitch-333333)](https://mozilla.github.io/pdf.js/web/viewer.html?file=https://raw.githubusercontent.com/cbgl/IROS-2024-presentations-resources/master/cbgl_pitch/presentation/IROS24_0101_presentation.pdf) [![youtube.com](https://img.shields.io/badge/2'_presentation-YouTube-FF0000)](https://www.youtube.com/watch?v=xaDKjI0WkDc) [![youtube.com](https://img.shields.io/badge/In_depth-YouTube-FF0000)](https://www.youtube.com/watch?v=TvTNEDGp-NU)

</div>

`cbgl` localises a 2D lidar sensor in a known 2D occupancy grid under global uncertainty in both position and orientation, from a single scan, in one shot. It needs no motion and no initial guess: give it a map, give it one scan, ask it once, and it tells you where the sensor was.

This is the ROS 2 version. The ROS 1 Kinetic package is on the `kinetic-devel` branch, where it has since been repaired, and [Upgrading from the ROS 1 version](#upgrading-from-the-ros-1-version) lists every name and every behaviour that changed.

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
  * [Refinement](#refinement)
    * [ICP settings](#icp-settings)
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
* A 2D lidar. One that sees **all the way round** is what this is built for and what every recorded figure was taken with. A narrower one is supported: the map is read all the way round at the sensor's own angular resolution and only the part of it the sensor could have seen is compared. The scan must say what it covers, through `angle_min` and `angle_increment`, and must be centred on the sensor's heading; one that is not is refused at the point of use rather than answered wrongly.

## Installation

### With Docker

The image carries both this package and `fsm_localisation`, so the build context spans the directory that holds both repositories rather than this one alone.

```sh
git clone git@github.com:cbgl/cbgl.git
git clone git@github.com:fourier-scan-matcher/fsm-localisation.git
cd cbgl/docker
docker compose build
```

The image runs a build, so the two arithmetic build options described below reach it as build arguments rather than as CMake arguments. Both belong to `fsm_localisation` and this package follows whatever that package was built with, so there are two arguments rather than four:

```sh
docker compose build \
  --build-arg FSM_LOCALISATION_EXACT_FP=ON \
  --build-arg FSM_LOCALISATION_DETERMINISTIC_FFT=ON
```

Launching puts the node in stand-by, which is what the image does by default. For a shell instead:

```sh
docker compose run --rm cbgl bash
```

### From source

`fsm_localisation` must be in the same workspace. Everything else is declared in the manifests and `rosdep` fetches it, bar two keys described below.

`csm` comes first, because `cbgl` will not configure without it. That is Andrea Censi's canonical scan matcher, which the ICP refinement uses. It has no `rosdep` rule and no release for any ROS 2 distribution, so it is built from source and installed to the system. Clone it outside the workspace: it carries a manifest of its own and `colcon` would otherwise try to build it as a workspace package.

```sh
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:cbgl/cbgl.git
git clone git@github.com:fourier-scan-matcher/fsm-localisation.git

cd ~
git clone https://github.com/AndreaCensi/csm.git
cd csm
git checkout 04c4a31072e8690bd0ff14de50be20c5b6b43e66
for p in ~/ros2_ws/src/cbgl/docker/csm_*.patch; do
  patch -p1 --forward --batch --fuzz=0 < "${p}"
done
cmake -S . -B build -DBUILD_SHARED_LIBS=ON -DCMAKE_CXX_FLAGS=-O2 \
  -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build
sudo cmake --install build
sudo rm -f /usr/local/lib/libcsm-static.a
sudo ldconfig

cd ~/ros2_ws
sudo apt install libfftw3-dev
rosdep install --from-paths src --ignore-src -r -y --skip-keys "libfftw3 csm"
colcon build --packages-up-to cbgl
```

`rosdep` needs `libfftw3` skipped because the key resolves through the upstream rules to a package Ubuntu dropped in favour of one per floating point precision, so `libfftw3-dev` is installed by name instead. It needs `csm` skipped because there is no rule for that key at all, and the step above has already installed it.

Every line of the `csm` recipe earns its place, so none of it is optional:

* **The commit is pinned** because the three patches are written against the sources exactly as that commit stands. The branch has not moved since August 2022, so treat the patches as permanent rather than as something upstream will take off your hands.
* **The three patches** are carried in `docker/`, each with its reasoning at its head. The first is the only one `csm`'s own build refuses to proceed without. The second corrects a write one entry past the end of a buffer inside `csm`, on the code path `do_alpha_test` reaches; leave it out and the refinement corrupts memory the moment that setting is switched on, which the settings the ROS 1 package ships do. The third makes `csm`'s own `pkg-config` file name the shared library rather than the static one.
* **`BUILD_SHARED_LIBS=ON` is a licensing requirement, not a preference.** `csm` is LGPL v3 and `cbgl` links it dynamically to satisfy that licence's relinking requirement. Without the switch `csm` builds an archive and `-lcsm` resolves to it silently. `cbgl` refuses to configure against a static `csm` rather than let that pass, but being told after the fact is not the point.
* **The optimisation flag is given on its own** rather than by asking for a release build. `csm`'s build file names no default build type, so without it the whole library compiles unoptimised. A release build would fix that and also define `NDEBUG`, which disarms the assertions guarding `csm`'s own fixed-size pools.
* **The static archive is removed** after the install because nothing should reach it, deliberately or otherwise.

If you skip the second patch, `colcon test` catches it: the refinement's own tests exercise the settings the ROS 1 package ships, which switch that path on.

Two build options exist for comparing builds rather than for running a robot. Both cost speed, both default off, and **both belong to `fsm_localisation` rather than to this package.** `FSM_LOCALISATION_EXACT_FP` builds at `-O2 -fno-fast-math -ffp-contract=off` instead of `-Ofast`, so that two compilers agree. `FSM_LOCALISATION_DETERMINISTIC_FFT` makes the matcher's transform plans chosen by heuristic rather than by timing, so that two runs agree.

```sh
colcon build --packages-up-to cbgl --cmake-args \
  -DFSM_LOCALISATION_EXACT_FP=ON -DFSM_LOCALISATION_DETERMINISTIC_FFT=ON
```

**This package carries no switch of its own, and that is deliberate.** The matcher is a header this package compiles for itself, and `fsm_localisation` compiled the same header when it was built. Two arithmetics over one header put two definitions of one inline function into a single process, and which of them survives linking is the linker's choice rather than anybody's intention. So `fsm_localisation` records what it was built with and this package reads it: where the two are built together the answer is the invocation above, and where `fsm_localisation` arrives already built, from a package manager, its arithmetic is settled and nothing here can revisit it. A `fsm_localisation` too old to carry that record is refused at configure time rather than built against.

### As a dependency of your own package

Five libraries are exported. Link the narrowest one that does what you need.

| Target | Knows about | Use it for |
| --- | --- | --- |
| `cbgl::cbgl_core` | nothing of ROS, not even a message type | the algorithm itself, given a grid and a scan |
| `cbgl::cbgl_grid` | occupancy grid messages, no `rclcpp` | turning a map message into the free space the search draws from |
| `cbgl::cbgl_map_file` | occupancy grid messages and `yaml-cpp`, no `rclcpp` | reading a map description file and its PGM into a map message |
| `cbgl::cbgl_icp_refinement` | the canonical scan matcher and no ROS | refining one pose against a map by ICP |
| `cbgl::cbgl_interface` | everything ROS | embedding the node in a process of your own |

`cbgl_core` links `cbgl_icp_refinement`, so `csm` is a load-time dependency of **every** target above whatever `refinement_method` is set to. See [Licensing](#licensing).

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

### Taking the map from a file instead of a topic

Empty `map_topic` and point `map_file` at a map description file on disk. The node then subscribes to no map topic at all.

```yaml
/**:
  ros__parameters:
    map_topic: ""
    map_file: "/maps/room.yaml"
```

The description is the format a map server reads:

```yaml
image: room.pgm
resolution: 0.05
origin: [-1.35, 2.70, 0.60]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

`image`, `resolution` and `origin` are required. `negate`, `occupied_thresh` and `free_thresh` default to `0`, `0.65` and `0.196`, and anything else in the file is ignored. A relative image path is resolved against the directory the description itself sits in.

The image is a **PGM**, either the binary `P5` variant or the plain `P2` one, which is what ROS map saving writes. No other format is read: this package links no image library and neither does its ROS 1 counterpart, so PGM is the one format both of them can be held to.

A picture is read on its own terms. The maximum sample value it declares is what its samples are rescaled from, so a picture declaring 15 means its 15 to be white exactly as a saved map means its 255, and a maximum above 255 is two bytes a sample with the more significant byte first. A maximum of nought, a maximum above 65535, and a sample above the maximum its own picture declares are each refused by name.

The file is read once, at start-up, and the map it describes is then the map for the life of the node. A description that cannot be read, a key that is missing, an image in another format and a map nothing can be localised in are each a refusal to start, naming the file and the key, rather than a node that comes up and refuses every request afterwards.

Naming both settings is allowed: the topic wins, the file is ignored, and a warning says so. Naming neither is refused at start-up.

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
| in | `map_topic` | `nav_msgs/msg/OccupancyGrid` | the map to localise in. Subscribed transient local at depth one, so a map latched before the node started arrives on subscription. Empty subscribes nothing and the map is read from `map_file` instead |
| out | `output_pose_topic` | `geometry_msgs/msg/PoseWithCovarianceStamped` | the answer, as a pose of the **robot**, in `fixed_frame_id`, stamped from the scan it was computed from |
| out | `hypotheses_topic` | `geometry_msgs/msg/PoseArray` | optional. Every hypothesis the search started from |
| out | `top_hypotheses_topic` | `geometry_msgs/msg/PoseArray` | optional. The survivors of the sift, in ranking order |
| out | `map_scan_topic` | `sensor_msgs/msg/LaserScan` | optional. The scan the answer would see, cast from the map, for laying over the scan the sensor did see |
| out | transform | `fixed_frame_id` <- `odom_frame_id` | only when `tf_broadcast` |

One service, `global_localisation`, of type `std_srvs/srv/Trigger`.

## Parameters

Every setting below lives in one file, `config/params.yaml`, keyed by the wildcard `/**:` so that renaming the node cannot silently drop the lot. Each is **read once, at start-up, and is read only thereafter**: setting one at runtime is refused rather than accepted and quietly ignored.

A value outside its permitted range is refused at start-up. The node says which setting and which value it was given, and exits with 1.

The **Default** column is what the node declares. The **Shipped** column is what `config/params.yaml` sets, which differs where reproducing the ROS 1 package's behaviour asks for it.

### Topics

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `scan_topic` | where 2D panoramic scans are published | `/scan` | `/scan` |
| `output_pose_topic` | where the answer is published | `/initialpose` | `/initialpose` |
| `hypotheses_topic` | optional. Every hypothesis drawn. Empty publishes nothing | empty | empty |
| `top_hypotheses_topic` | optional. The survivors of the sift. Empty publishes nothing | empty | empty |
| `map_scan_topic` | optional. The scan the answer would see. Empty publishes nothing | empty | empty |

### Where the map comes from

One of the two settings below has to name something. Both empty is refused at start-up, and both named is the topic with a warning that the file was ignored.

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `map_topic` | the topic the map arrives on. Empty subscribes nothing | `/map` | `/map` |
| `map_file` | a map description file on disk, read once at start-up. Empty reads no file | empty | empty |

### Frames

Frame names carry no leading slash. tf2 rejects one, and one is refused at start-up.

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `fixed_frame_id` | the frame the map and the published answer are expressed in | `map` | `map` |
| `base_frame_id` | the robot's own frame, which the published pose places | `base_footprint` | `base_footprint` |
| `odom_frame_id` | the odometry frame, read only when broadcasting | `odom` | `odom` |
| `tf_broadcast` | broadcast `fixed_frame_id` <- `odom_frame_id` for each answer | `false` | `true` |
| `tf_lookup_timeout` | seconds to wait for a transform before refusing a run | `0.1` | `0.1` |

The sensor's own frame is **not** a parameter. It is read off the scan message, and the node works out where the sensor sits on the robot by looking `base_frame_id` up against it. That transform must exist in the tree or every run is refused for want of it.

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

### Refinement

Every survivor of the sift is refined against the map. Two algorithms can do it and this chooses which.

| Parameter | Meaning | Default | Shipped |
| --- | --- | --- | --- |
| `refinement_method` | which algorithm refines a survivor: `fsm`, `icp` or `none` | `fsm` | `fsm` |

`fsm` is the panoramic scan matcher of `fsm_localisation`, which reads the map afresh at every pose it tries. `icp` is Andrea Censi's canonical scan matcher, point to line, which reads the map **once**, at the survivor's own pose, and then matches the measurement onto that one frozen reading. That difference is the whole of what separates them: one consults the map throughout, the other consults it once.

`none` refines nothing and publishes the sift's own best candidate as it stands. Nothing casts a second time, so it is by far the cheapest configuration and by far the least accurate: the answer is quantised to what the sift can express, one of `da` orientations at the position of a drawn free cell. Measured on the real map at two hundred trials:

| Refinement | Within 5 cm | Within 50 cm | Median | Heading | Time |
| --- | --- | --- | --- | --- | --- |
| `fsm` | 100% | 100% | 0.9 mm | 0.02 deg | 167 ms |
| `none` | 7% | 94.5% | 150 mm | 1.71 deg | 96 ms |

So `none` puts the robot in the right part of the building and no more. It is what a caller wants who refines elsewhere, or who needs a room rather than a pose. Giving solver settings alongside it is refused rather than one of the two quietly winning.

The ROS 1 package offered both refinements and shipped with `icp` selected. This ships with `fsm`, so a configuration that says nothing gets what this package has always done. Any other value is refused at start-up, naming the setting and the value.

`icp` needs `csm`, which is not installed by a package name and is LGPL v3. See [Installation](#from-source) for building it and [Licensing](#licensing) for what depending on it means.

**A sensor that cannot see all the way round works with either refinement.** What supports such a sensor is the sift: it reads the map all the way round at the sensor's own angular resolution, which is more readings than the sensor has, and compares only the middle window of the sensor's own length. The refinement then decides whether the answer survives the last stage, and both refinements now follow the sensor's own aperture rather than assuming a whole turn.

This page used to say that a narrower sensor wanted `icp`, because `fsm` matched a full turn against a full turn. That was a defect in the scan matcher path rather than a property of it: nothing told that refinement what the sensor covered, so it read the map over a whole turn and matched it against three quarters of one, which converges on a confident wrong answer rather than failing. It is repaired. Measured on the real map at a thousand trials, three quarters of a turn, the scan matcher answers 99.6 per cent of trials inside half a metre and five degrees with a median error of 0.8 mm, against the solver's 99.8 per cent and 4.3 mm, and it is the quicker of the two.

#### ICP settings

Declared only when `refinement_method` is `icp`, and refused when out of range, naming the setting and the value. Every name and every default is the ROS 1 package's own.

**The Shipped column here means something different from every other table on this page.** Elsewhere it is what `config/params.yaml` sets. This package's own settings file sets none of these, so the column is what the ROS 1 package's `params_csm.yaml` carries, printed in the type this package declares.

**That file does not transfer as it stands, and the types are why.** It writes ten of these as `0` or `1` where they are declared here as booleans: `debug_verify_tricks`, `do_alpha_test`, `do_compute_covariance`, `do_visibility_test`, `outliers_remove_doubles`, `restart`, `use_corr_tricks`, `use_ml_weights`, `use_point_to_line_distance` and `use_sigma_weights`. Write those as `true` and `false`. For the same reason a decimal setting must be written as one: `do_alpha_test_thresholdDeg: 5.0` rather than `5`, exactly as `dl` must be written `15.0`. A whole number against a decimal declaration is refused by the parameter machinery in its own words rather than in this package's, so the sentence you get names neither the value nor what to do about it.

| Parameter | Default | Shipped |
| --- | --- | --- |
| `clustering_threshold` | `0.25` | `1.025` |
| `debug_verify_tricks` | `false` | `false` |
| `do_alpha_test` | `false` | `true` |
| `do_alpha_test_thresholdDeg` | `20.0` | `5.0` |
| `do_compute_covariance` | `false` | `false` |
| `do_visibility_test` | `false` | `false` |
| `epsilon_theta` | `0.000001` | `0.00001` |
| `epsilon_xy` | `0.000001` | `0.00001` |
| `max_angular_correction_deg` | `45.0` | `180.0` |
| `max_correspondence_dist` | `0.3` | `1.0` |
| `max_iterations` | `10` | `100` |
| `max_linear_correction` | `0.50` | `10.0` |
| `orientation_neighbourhood` | `20` | `20` |
| `outliers_adaptive_mult` | `2.0` | `2.0` |
| `outliers_adaptive_order` | `0.7` | `1.0` |
| `outliers_maxPerc` | `0.90` | `1.0` |
| `outliers_remove_doubles` | `true` | `false` |
| `restart` | `false` | `false` |
| `restart_dt` | `1.0` | `0.01` |
| `restart_dtheta` | `0.1` | `0.01` |
| `restart_threshold_mean_error` | `0.01` | `0.0001` |
| `sigma` | `0.010` | `0.01` |
| `use_corr_tricks` | `true` | `false` |
| `use_ml_weights` | `false` | `false` |
| `use_point_to_line_distance` | `true` | `true` |
| `use_sigma_weights` | `false` | `false` |

Measured on the real map at two hundred trials a side, the two refinements cost 167 ms and 232 ms at a full turn and 188 ms at three quarters of one, against a node that reports its own work rather than the round trip. Accuracy is level: every trial of both inside five centimetres at a full turn.

The ROS 1 package reads three more, `gpm_theta_bin_size_deg`, `gpm_extend_range_deg` and `gpm_interval`. They belong to a different algorithm in the same library, one this package does not call, and the variant built here has no such settings at all. They are not carried and setting them does nothing.

`use_sigma_weights` is refused when true. It weights each correspondence by a per-reading uncertainty that nothing fills in, so switching it on cannot work and the refusal says so rather than letting the solver complain once per reading.

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

**A sensor that does not see all the way round needs this changed.** The shipped `360` is a full turn's worth of rays, so a sensor covering three quarters of one at the same spacing carries 270 and is refused as too short, with a message that names the ray count and not the aperture. Set `size_scan: 0` to take every ray the sensor sends, or to that sensor's own ray count. Nothing derives it for you, and the refusal will not tell you the aperture is why.

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

The ROS 1 Kinetic package is on `kinetic-devel`. This version is not a drop-in replacement.

**That branch has moved, and which commit you check a behaviour against now matters.** Between 1 July 2024 and 25 February 2026 it could not localise with the configuration it shipped, and its tip has since been repaired. So there are three states worth telling apart:

| State | What it does |
| --- | --- |
| the current tip of `kinetic-devel` | five defects repaired, and it localises. This is what a reader should compare against unless they want the history |
| `49bf545` of 25 February 2026 and back to 2 July 2024 | cannot cast a ray with the `map_scan_method` its own configuration selects, so it cannot localise as it ships. The alternative scanner that parameter offers avoids the broken code entirely and works, so those states ship misconfigured rather than incapable, and the difference is one value in one file |
| `fb9dab1` of 15 May 2024 | the last state before the faults. Its caster is byte identical to the code the published experiments were run with, and its own sources had stood unchanged for 208 days by then. This is the commit to use for a comparison against the published work |

The repairs to the tip were the split caster constructor, a conversion buffer sized on the stack from the map, the hypothesis sampling interval, the map origin's dropped orientation and a grid cast that never subtracted the origin. Two of those bore on maps away from the world origin, which the ROS 2 version handles and those older states did not.

**One behaviour to know about if you compare figures.** A figure measured on a state in that window says nothing about the current tip, and one measured on the tip says nothing about that window. Neither substitutes for the other, and whichever is quoted has to name the state it came from.

**Renamed or moved**

| ROS 1 | ROS 2 |
| --- | --- |
| service `global_localization`, `std_srvs/Empty`, node relative under a configurable leaf | service `global_localisation`, `std_srvs/srv/Trigger`, node relative and fixed |
| node `cbgl_node` in namespace `robot`, so the service answered at `/robot/cbgl_node/global_localization` | node `cbgl` in namespace `/`, so it answers at `/cbgl/global_localisation`. Both are launch arguments |
| `configuration_files/params_cbgl.yaml`, `params_csm.yaml`, `params_fsm.yaml` | one `config/params.yaml`, keyed `/**:` |
| `fixed_frame_id`, `base_frame_id`, `odom_frame_id` | the same three names, unchanged. They were renamed once during the port and renamed back, because a frame setting that quietly means something else is the kind of change that places every answer for the wrong frame on a robot whose own frame is not the default |
| `launch/cbgl.launch` | `launch/cbgl.launch.xml` |

**Gone**

| Setting | Why |
| --- | --- |
| `do_icp`, `do_fsm` | replaced by `refinement_method`, one setting with one value, where two booleans could say both or neither. **The ROS 1 package shipped `do_icp: true`**, so anyone running its shipped configuration was refining by ICP; that path exists here and is selected by `refinement_method: "icp"`, but this package ships with the scan matcher instead |
| `laser_z_orientation` | it existed to compensate for reading the grid's rows in reverse. The replacement map handles the origin's translation, its rotation and its row order itself. The ROS 1 build now handles a rotated origin too, so this setting is a leftover there rather than a difference in what the two can do |
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
* **Hypotheses are drawn on free cells.** This version disperses by free cell through the map's own accessor and computes no sampling interval at all. The ROS 1 version computed one, and computed it with the map's centre and its half extent the wrong way round, which was latent on a map at the world origin and left the rejection loop spinning on a map centred there. **That is repaired on `kinetic-devel` and this is no longer a difference between the two builds**, only a difference in how each arrives at the same place. It is recorded because a reader comparing against a ROS 1 state older than the repair will meet it.
* **The poses drawn are sensor poses.** The ROS 1 version dispersed robot poses over free cells and offset to the sensor afterwards, which can place the sensor, which is what casts, on a blocked cell.
* **No pose is published when a run fails.** The ROS 1 version published one regardless, reading its winner from index minus one. It never refused a candidate, scoring every one unconditionally; the index stayed at minus one when the survivor list was empty, which `top_k_caers` at zero produces.
* **The published pose is stamped from the scan** that produced it, not from the wall clock, and its frame is `fixed_frame_id` rather than the hardcoded string `/map`.
* **A parameter outside its range refuses to start.** The node says which setting and which value, and exits with 1.

## Licensing

`cbgl`'s own sources are MIT. See [LICENSE](LICENSE). That is unaffected by everything below: MIT code may link a library under any of these terms.

What a **redistributed binary** carries is a different question, and the answer is that it carries obligations from three directions at once.

| From | Through | Terms |
| --- | --- | --- |
| CGAL | `fsm_localisation` | GPL v3 |
| FFTW | `fsm_localisation` | GPL v2 or later |
| the ray casting inside `fsm_localisation` | `fsm_localisation` | Apache 2.0 |
| `csm` | the `icp` refinement | LGPL v3 |

The ROS 1 version also carried LGPL v2.1 and Apache 2.0 obligations from amcl's map and particle filter and from a vendored ray casting library. Both are gone from these sources along with the code that brought them.

FFTW's terms allow version 3, so the two GPL rows combine to GPL v3 rather than conflicting. That is worth saying rather than leaving a reader to check it.

**Adding `csm` costs a redistributor no new source obligation.** GPL v3 already applied through CGAL before this package gained an ICP refinement, and GPL v3 is stricter than LGPL v3 on that point: a redistributor who supplies the corresponding source of the whole work has already done more than LGPL v3's relinking clause asks. That is the reason the dependency was taken.

**It does add two paperwork obligations, and those are not subsumed.**

* A notice with each copy of the combined work saying that `csm` is used and that `csm` and its use are covered by the LGPL. This section is that notice.
* The LGPL v3 text alongside the work. GPL v3 already required its own text to travel, so that half was owed already; the LGPL text was not. Both are here: [COPYING](COPYING) for the GPL v3 and [COPYING.LESSER](COPYING.LESSER) for the LGPL v3. The package carries its own copies of all three and installs them into `share/cbgl`, so they travel with a built tree rather than only with these sources. Both images additionally carry `csm`'s own copy at `/usr/local/share/licenses/csm/`.

**`csm` is linked dynamically, and that is a requirement rather than a preference.** LGPL v3's relinking condition is met by dynamic linking and is awkward otherwise. The build refuses to configure against a statically built `csm` rather than let a static link happen quietly.

**`csm` is carried modified.** Three patches are applied, listed under [Installation](#from-source) and kept in `docker/` with their reasoning. What a redistributor must be able to supply is therefore the pinned upstream commit **and those patches**, not upstream alone.

**One consequence worth stating plainly.** Anybody who later removes the CGAL dependency to shed GPL v3 will find that the LGPL v3 obligation does not go with it. It becomes the binding constraint at that point rather than a detail underneath a stricter one.

This is a reading of the licences and not legal advice. If a binary is ever redistributed commercially it wants a real review by somebody qualified to give one.

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
