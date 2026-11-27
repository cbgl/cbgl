# Parameters in detail

This page gives the reasons behind several settings and their history. The tables of every setting, with its meaning and its default, are in [Parameters](../README.md#parameters) in the README.

## Frames

### The sensor's frame

The sensor's own frame is not a parameter. The node reads it from the scan message, and it finds where the sensor sits on the robot by looking `base_frame_id` up against it. The same transform also says whether the sensor is mounted upside down, with its z axis pointing down, so no setting is needed for that.

### `tf_broadcast`

`tf_broadcast` defaults to `false`, because broadcasting into somebody else's transform tree is intrusive. The shipped file turns it on, which is what the ROS 1 package shipped.

## Refinement

### The canonical scan matcher was removed

The ROS 1 package offered a second refinement, Andrea Censi's canonical scan matcher, and shipped with it selected. **That path is not carried here.** It was carried, and it was removed after the scan matcher was measured against it.

The measurement used three quarters of a turn and a thousand trials. The scan matcher answers 99.6 per cent of trials inside half a metre and five degrees, with a median error of 0.8 mm. The canonical scan matcher answers 99.8 per cent, with a median error of 4.3 mm. The scan matcher is also the quicker of the two. At a full turn, the scan matcher was ahead on every measure.

The canonical scan matcher cost an LGPL v3 dependency with no ROS 2 release. It also cost three patches against a commit that upstream has not moved since 2022. One of those patches corrected a write past the end of a buffer. Nothing that the canonical scan matcher did was worth that cost.

### Sensors that do not see all the way round

A sensor that cannot see all the way round is supported with either value of `refinement_method`. The sift is what supports such a sensor. The sift reads the map all the way round, at the angular resolution of the scan it works from. That resolution is rounded to the nearest spacing that divides a turn evenly. That gives more readings than the scan has. The sift then compares only the middle window of those readings, as long as the scan. That window is the part of the map that the sensor could have seen.

With `size_scan` at `0`, as shipped, the sift works from the sensor's own scan. So the map is read at the sensor's own angular resolution. With any other value, the scan is first reduced to `size_scan` rays. The map is then read at that coarser spacing, which is the aperture of the scan divided by `size_scan`.

With `fsm`, the refinement and the final selection also cast a whole turn and compare only that middle window. With `none`, nothing is refined, and the best candidate of the sift is the answer.

The scan must state what it covers through `angle_min` and `angle_increment`, and it must be centred on the sensor's heading. A scan that is not centred is refused at the point of use. [The scan is not centred](troubleshooting.md#the-scan-is-not-centred) explains why.

### What this documentation used to say

This documentation used to say that a narrower sensor needed the other refinement, because the scan matcher matched a full turn against a full turn. That was a defect in the scan matcher path, not a property of it. Nothing told that refinement what the sensor covered. So it read the map over a whole turn and matched it against three quarters of one. That converges on a confident wrong answer instead of failing. The defect is repaired, and the repair left the second refinement with nothing to do.

## Matcher

### `size_scan`

`size_scan` is how many rays a scan is reduced to, once, before the sift. The sift, the refinement and the selection all work from that same reduced scan. Each of them casts a whole turn from the map, at the angular spacing of the reduced scan. It then compares only the middle of that cast, as wide as the reduced scan. `0` uses every ray the scan carries. Any other value refuses a scan that carries fewer rays than that value.

**The shipped `0` refuses no scan for its ray count.** It takes every ray the scan carries, whatever the aperture of the sensor. The shipped value used to be `360`, a full turn of rays. So a sensor that covered three quarters of a turn at the same spacing carried 270 rays, and it was refused as too short. Nothing derives the value from the aperture, and the refusal names `size_scan`, not the aperture.

### Why some shipped values differ from the defaults

`xy_bound` is shipped at 1.2 m, against the matcher's own default of 0.2 m. A survivor of the sift stands at the centre of a cell, and it can be a cell or more from the truth.

`enforce_early_gearup`, `enforce_translation_bound` and `recover_on_failed_sift` are behaviours that are **unconditional** in the matcher that this algorithm originally shipped with. They are optional in the matcher that it now borrows. The shipped file turns all three on, because leaving them off would be refining with a different matcher and calling it the same one.

### `ray_search`

`ray_search` defaults to `angular`, which did not exist when the ROS 1 package was written. `windowed` is what that package used. It is kept so that a run can be compared against results published before `angular` existed.
