# Upgrading from the ROS 1 version

The ROS 1 Kinetic package is on the `kinetic-devel` branch. This version is not a drop-in replacement. This page lists every name and every behaviour that changed between the two. [Parameters](../README.md#parameters) in the README lists the settings of this version.

## Which ROS 1 state to compare against

**The `kinetic-devel` branch has moved, so the commit that you check a behaviour against matters.** Between 1 July 2024 and 25 February 2026, the branch could not localise with the configuration that it shipped. Its tip has since been repaired. So three states are worth telling apart:

| State | What it does |
| --- | --- |
| the current tip of `kinetic-devel` | five defects repaired, and it localises. Compare against this state unless you want the history |
| `49bf545` of 25 February 2026 and back to 2 July 2024 | cannot cast a ray with the `map_scan_method` that its own configuration selects, so it cannot localise as it ships. The alternative scanner that this parameter offers avoids the broken code entirely and works. So these states ship misconfigured, not incapable, and the difference is one value in one file |
| `fb9dab1` of 15 May 2024 | the last state before the faults. Its caster is byte identical to the code that the published experiments were run with, and its own sources had stood unchanged for 208 days by then. Use this commit for a comparison against the published work |

The repairs to the tip were these:

* the split caster constructor
* a conversion buffer sized on the stack from the map
* the hypothesis sampling interval
* the dropped orientation of the map origin
* a grid cast that never subtracted the origin

Two of those repairs bore on maps away from the world origin, which the ROS 2 version handles and those older states did not.

**One behaviour to know about if you compare figures.** A figure measured on a state in that window says nothing about the current tip. A figure measured on the tip says nothing about that window. Neither substitutes for the other. Whichever figure is quoted must name the state that it came from.

## Renamed or moved

| ROS 1 | ROS 2 |
| --- | --- |
| service `global_localization`, `std_srvs/Empty`, node relative under a configurable leaf | service `global_localisation`, `std_srvs/srv/Trigger`, node relative and fixed |
| node `cbgl_node` in namespace `robot`, so the service answered at `/robot/cbgl_node/global_localization` | node `cbgl` in namespace `/`, so it answers at `/cbgl/global_localisation`. Both are launch arguments |
| `configuration_files/params_cbgl.yaml`, `params_csm.yaml`, `params_fsm.yaml` | one `config/params.yaml`, keyed `/**:` |
| `fixed_frame_id`, `base_frame_id`, `odom_frame_id` | the same three names, unchanged. They were renamed once during the port and renamed back, because a frame setting that quietly means something else places every answer for the wrong frame on a robot whose own frame is not the default |
| `launch/cbgl.launch` | `launch/cbgl.launch.xml` |

## Removed settings

| Setting | Why |
| --- | --- |
| `do_icp`, `do_fsm` | replaced by `refinement_method`, one setting with one value, where two booleans could say both or neither. **The ROS 1 package shipped `do_icp: true`**, so anyone running its shipped configuration was refining by the point to line solver. That path is not carried here at all. The scan matcher was measured against it and matches or beats it at every aperture, so the dependency it needed was dropped, not ported |
| `laser_z_orientation` | in the ROS 1 build it says which way the sensor's z axis points. Its shipped value, `upwards`, compensates for that build reading the grid's rows in reverse, and the replacement map handles the origin's translation, its rotation and its row order itself. `downwards` serves a sensor mounted upside down. This version reads that from the sensor's placement in the transform tree, so there is nothing to set. The ROS 1 build still needs `downwards` set for such a sensor |
| the `vanilla` value of `map_scan_method` | it selected a second ray tracer that is gone. The three surviving names are `ray_marching`, `bresenham` and `cddt`, and any other name is refused at start-up. `vanilla` was also what ROS 1 fell back to when the setting was absent, so a configuration that never named one must name one now |
| `do_undersample_scan`, `undersample_rate` | `size_scan` does this, and does it by resampling, not by dropping every nth ray |
| `publish_pose_sets` | replaced by three optional topics, each off while its name is empty |
| `global_localisation_service_name` | the service is node relative and fixed |
| `pkg_name` | the logger already provides a prefix |

## Behaviour that changed

* **The service waits.** It returns when the work is done, and it carries the outcome. The ROS 1 service returned immediately and reported nothing about the run.
* **The refinement matches against the map.** In the ROS 1 version, the scan matching path matched each survivor against one scan manufactured at the pose of that survivor. This version casts into the map at every pose it tries. This is the largest behavioural difference in the port, and the one most likely to move the accuracy figures.
* **`tf_broadcast: false` works.** In the ROS 1 version, with broadcasting off, the sensor's placement on the robot was never looked up. Every cast composed the robot pose with an uninitialised transform, so the published pose was arbitrary. This version reads the placement on every run, whether broadcasting is on or off. That is why the node's own default for `tf_broadcast` can be false.
* **A rotated cast is labelled with the orientation that the rotation amounts to.** The ROS 1 version shifted the range vector by one number of samples. It then labelled the result with a different angle. The error grew to nearly eight degrees by the last candidate, in the opposite direction. So the refinement started from a pose that disagreed with the scan it was chosen for.
* **Hypotheses are drawn on free cells.** This version disperses by free cell, through the map's own accessor, and computes no sampling interval at all. The ROS 1 version computed one, with the map's centre and its half extent the wrong way round. That was latent on a map at the world origin, and it left the rejection loop spinning on a map centred there. **That is repaired on `kinetic-devel`, and it is no longer a difference between the two builds.** The two builds differ only in how each arrives at the same place. It is recorded because a reader who compares against a ROS 1 state older than the repair will meet it.
* **The poses drawn are sensor poses.** The ROS 1 version dispersed robot poses over free cells and offset them to the sensor afterwards. That can place the sensor, which is what casts, on a blocked cell.
* **No pose is published when a run fails.** The ROS 1 version published one regardless, and read its winner from index minus one. It never refused a candidate, and it scored every one unconditionally. The index stayed at minus one when the survivor list was empty, which `top_k_caers` at zero produces.
* **The published pose is stamped from the scan** that produced it, not from the wall clock. Its frame is `fixed_frame_id`, not the hardcoded string `/map`.
* **A parameter outside its range refuses to start.** The node names the setting and the value, and exits with 1.
