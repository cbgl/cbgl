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

| i/o | Type                                      | What                                                                                                              |
| --- | ----------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| in  | `sensor_msgs/LaserScan`                   | The scan message received via topic `configuration_files/scan_topic`                                              |
| in  | `nav_msgs/OccupancyGrid`                  | The map received via topic `configuration_files/map_topic`                                                        |
| out | `geometry_msgs/PoseWithCovarianceStamped` | The output pose published through topic `configuration_files/output_pose_topic`                                   |
| out | Transform                                 | The transform between the `odom` frame and the `map` frame if `configuration_files/tf_broadcast` is set to `true` |

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
