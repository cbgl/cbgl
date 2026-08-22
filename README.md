<h1 align='center' style="text-align:center; font-weight:bold; font-size:2.0em;letter-spacing:2.0px;"> CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor </h1>

<div align="center">

[![ieeexplore.ieee.org](https://img.shields.io/badge/IEEE/RSJ_IROS_2024_paper-00629B)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10802235) [![github.com](https://img.shields.io/badge/pitch-333333)](https://mozilla.github.io/pdf.js/web/viewer.html?file=https://raw.githubusercontent.com/cbgl/IROS-2024-presentations-resources/master/cbgl_pitch/presentation/IROS24_0101_presentation.pdf) [![youtube.com](https://img.shields.io/badge/2'_presentation-YouTube-FF0000)](https://www.youtube.com/watch?v=xaDKjI0WkDc) [![youtube.com](https://img.shields.io/badge/In_depth-YouTube-FF0000)](https://www.youtube.com/watch?v=TvTNEDGp-NU)

</div>

<h3 align="center">
    <a href="#inputoutput-at-a-glance">I/O</a>
    <span> · </span>
    <a href="#pre-installation">install</a>
    <span> · </span>
    <a href="#run">run</a>
</h3>

## What this release repairs, and why a figure from before it does not carry over

**Read this before quoting any figure about this package.** This is the current tip: `master` and `kinetic-devel` both point here. It is not the code the paper above was run with. Five defects were repaired after commit `49bf545` of 25 February 2026, which was the tip until these landed, and they are grouped below into the four changes they take: the raycaster's split constructor, the conversion buffer on the stack, the sampling interval, the dropped origin orientation and the grid cast that never subtracted the origin.

Between 1 July 2024 and those repairs the package could not cast a ray with the configuration it ships. So a figure measured on any state in that window says nothing about this one, and a figure measured on this one says nothing about that window. Figures from the two are not interchangeable and neither is a substitute for the other; whichever is quoted has to name the state it came from.

What is repaired, in the order it matters:

1. **The raycaster could not build a grid, by either route into it.** A commit of 2 July 2024 moved the grid population out of the occupancy map constructor that takes a file name and into a new one that takes a decoded pixel buffer. The new one is handed a bare buffer, which carries no dimensions, and assigns neither the width nor the height; the old one calls it as an expression statement, which builds a temporary and discards it rather than delegating. Since that day the scan method the package selects in its own configuration, `ray_marching`, has had no grid corresponding to the map to cast against. The repair gives the buffer route its two dimensions, makes the file name route delegate, frees the buffer the decode allocated, and routes both through one function so that both leave the same invariant behind.

   If you are using the released package and cannot move to a repaired build, there is a one word change that avoids this entirely: set `map_scan_method` to `vanilla`. That route never touches this code, traces the occupancy grid message directly, and honours the map origin including its rotation. It is slower.

2. **The map callback sized a buffer on the stack from the map message**, at four bytes a cell, which puts any map of a realistic size past the stack limit and kills the node in the callback before any scan method is consulted. The buffer is now on the heap, which is where every published state before the tip held it.

3. **The hypothesis draw asked for the wrong interval.** For a map message origin `o` and an extent `W` it wants `[o, o + W]` and computed `[-o, W + o]`, which is the same interval with the origin's sign inverted. The two coincide where the origin is nought and the cell count even, part by the axis's first cell where the count is odd, and diverge with the origin otherwise: displaced enough, the interval inverts or leaves the map altogether, and the rejection loop that draws from it then draws, tests and never breaks.

4. **The map origin's orientation was dropped, and the grid cast never subtracted the origin at all.** The internal map recorded the message origin's position and not its orientation, and had no field to hold one, so the free cell test answered about an unrotated ghost of the room hinged at the same corner. Separately the cast into the raycaster's grid divided a world position by the resolution with no reference to the map origin. The internal map now carries the origin's orientation, the conversion macros carry a rotation term, the draw runs over the map's own axes, and the cast subtracts the origin and rotates into those axes. On a map at the world origin with no rotation every one of these computes what it computed before.

One interface changes with the fourth repair and it changes in the package's own headers: `MAP_WXGX`, `MAP_WYGY`, `MAP_GXWX` and `MAP_GYWY` each take both coordinates now rather than one. Under a rotation a cell's world x depends on its row as well as its column, and the two argument form cannot express the answer. Nothing inside this package called the first two; the other two are called in three places, all updated here.

A further defect named in the same investigation is **not** repaired, because it was already cured before these repairs began: a commit of 1 July 2024 declared the map conversion to return a byte vector and gave it no return statement, which aborted the node on every map, and a later commit cured that by making the function return nothing. It is recorded here so that a reader looking for it knows why there is nothing to find.

There is also a branch that repairs only the last two of the five, `repair/kinetic-fb9dab1-map-origin`, off commit `fb9dab1` of 15 May 2024. That one is **not** a released state and is not intended to become one. It exists because `fb9dab1` is the last state before the faults whose raycaster is intact, so it is the one state against which the origin repair can be measured with nothing else moving. Measured over ten configurations and a thousand trials, it and this state agree on 981 records to the last bit and their execution times differ by a median of 0.44 ms.


`cbgl` is a ROS package written in C++ that allows you to localise your 2D LIDAR sensor in a given 2D occupancy grid map under global uncertainty in position and orientation, in minimal time

- You can expect the execution time to roughly have an order of magnitude of $`10e \cdot \text{area} \cdot N_s`$ microseconds, where $\text{area}$ is the area of the map's free space and $`N_s`$ is the LIDAR's number of rays. (Strictly speaking the execution time varies according to the geometry of the environment and other factors.) In the video below the environment area is $`2000`$ m$`^2`$ and localisation is performed in under four seconds

- CBGL does not require motion for performing global localisation: it's a one-shot approach that only requires a single laser scan measurement and the map of the sensor's environment

<!--
Click on the image for a brief demo
[![CBGL in Willowgarage](https://img.youtube.com/vi/DkKdxFNJG4g/maxresdefault.jpg)](https://youtu.be/DkKdxFNJG4g)
-->

https://github.com/li9i/cbgl/assets/1658819/5794cd21-651d-4924-b453-25c46b9e42a9

<sub>A panoramic 2D LIDAR sensor mounted on a turtlebot 2 is spawned into
an environment at a pose whose immediate surroundings are repeated in (almost)
the same geometry and proportions at locations other than the sensor's spawning
ground. The user calls the global localisation service once before moving the
robot at a second challenging pose, at which she calls it for a second time.
Both times `cbgl` is successful in estimating the sensor's pose. Sensor
characteristics: $`N_s = 360`$ rays; noise: $`\sim \mathcal{N} (0.0, 0.05^2)`$ [m,m$`^2`$]</sub>

## Why use CBGL

<p align="center">
  <img src="https://i.imgur.com/kAD8AmS.png?1">
</p>

Table of Contents
=================

* [Pre-installation](#pre-installation)
* [Install](#install)
* [Run](#run)
  * [Launch](#launch)
  * [Call](#call)
* [Input/output at a glance](#inputoutput-at-a-glance)
* [Motivation](#motivation)
* [More results](#more-results)

## Pre-installation

`cbgl` is installed, launched, and called via Docker:

- if this is your first time running docker then I happen to find [this docker installation guide](https://youtu.be/SAMPOK_lazw?t=67) very friendly and easy to follow
- if instead you wish to install and run the package natively in Ubuntu 16.04, see the [INSTALLATION_DEPRECATED.md](https://github.com/li9i/cbgl/blob/master/INSTALLATION_DEPRECATED.md) guide.

## Install

Build the image with the most recent code of this repository with:

```sh
git clone git@github.com:li9i/cbgl.git
cd cbgl
docker compose build
```

or pull it:

```sh
docker pull li9i/cbgl:latest
```

## Run

### Launch

If you cloned the repository then you may run the image via `compose`

```sh
cd cbgl
docker compose up
```

or, in any case, you may run the image with

```sh
docker run -it \
    --name=cbgl \
    --net=host \
    --rm \
    li9i/cbgl:latest
```

### Call

Launching `cbgl` simply makes it go into stand-by mode and does not actually localise your sensor. To do so simply call the provided service

```sh
docker exec -it cbgl sh -c "source ~/catkin_ws/devel/setup.bash; rosservice call /robot/cbgl_node/global_localization"
```

## Input/output at a glance

| i/o | Type                                                                                                                                   | What                                                                                                                                                                          |
| --- | -------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| in  | [`sensor_msgs/LaserScan`](https://docs.ros.org/en/kinetic/api/sensor_msgs/html/msg/LaserScan.html)                                     | The scan message received via topic [`configuration_files/params_cbgl/scan_topic`](cbgl/configuration_files/params_cbgl.yaml#L14)                                             |
| in  | [`nav_msgs/OccupancyGrid`](https://docs.ros.org/en/kinetic/api/nav_msgs/html/msg/OccupancyGrid.html)                                   | The map received via topic [`configuration_files/params_cbgl/map_topic`](cbgl/configuration_files/params_cbgl.yaml#L9)                                                        |
| in  | a map description file                                                                                                                 | The map read from disk at start up, from the path [`configuration_files/params_cbgl/map_file`](cbgl/configuration_files/params_cbgl.yaml#L13) names, used only when `map_topic` is empty |
| out | [`geometry_msgs/PoseWithCovarianceStamped`](https://docs.ros.org/en/kinetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) | The output pose published through topic [`configuration_files/params_cbgl/output_pose_topic`](cbgl/configuration_files/params_cbgl.yaml#L17)                                  |
| out | Transform                                                                                                                              | The transform between the `odom` frame and the `map` frame if [`configuration_files/params_cbgl/tf_broadcast`](cbgl/configuration_files/params_cbgl.yaml#L6) is set to `true` |

### Where the map comes from

The map may arrive either way, and which one is decided once at start up:

- `map_topic` set, which is how the package ships: the map is taken from that topic and `map_file` is ignored, with a warning if it was set as well.
- `map_topic` empty and `map_file` set: the file is read once at start up and no subscription is made at all.
- both empty: the node refuses to start and says so, naming both settings.

`map_file` names a map description in the same format the ROS map server reads: a YAML carrying the image's filename, the resolution in metres per cell, the origin as x, y and yaw, and optionally `negate`, `occupied_thresh` and `free_thresh`. A relative image path resolves against the directory the YAML itself sits in. **The image must be a PGM**, either the binary `P5` or the text `P2` variant, and any other format is refused at start up with a message saying so. That is narrower than the map server, which accepts several formats through an image library; this package carries no such library and neither does its ROS 2 counterpart, so the two accept the one format ROS map saving writes by default rather than accepting different sets.

Either route produces the same map internally, because the file route builds the same message a topic would deliver and hands it to the same code. So a map's origin, including its rotation, is honoured identically whichever way it arrived.

## Motivation

<p align="center">
  <img src="https://i.imgur.com/LQBwg7G.png">
</p>

## More results

<p align="center">
  <img src="https://i.imgur.com/0qt3chL.png">
</p>

## Citation

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
