# Known gaps and deferred work

Recorded while this package was carried from ROS 1 Kinetic to ROS 2 Lyrical, and while its own copies of the scan matcher and the ray casting were replaced by `fsm_localisation` taken as a dependency. Nothing here stops the package answering. Every gap was left alone deliberately, and the reason is given so that the decision can be revisited rather than rediscovered. The last section is the exception in kind rather than in treatment: it holds three questions that a comparison against the ROS 1 build was meant to settle and has not yet been run to settle.

Table of contents
=================

* [The algorithm](#the-algorithm)
* [What the sensor has to be](#what-the-sensor-has-to-be)
* [The node](#the-node)
* [What the tests do not cover](#what-the-tests-do-not-cover)
* [Build, packaging and linting](#build-packaging-and-linting)
* [Three questions the comparison against the ROS 1 build has still to settle](#three-questions-the-comparison-against-the-ros-1-build-has-still-to-settle)

## The algorithm

### The sift sees holes in a scan that the refinement does not

A scan is reduced once, up front, and the same reduced scan feeds the sift, the refinement and the selection. But the gap filling happens inside the matcher, not before it. So a scan with a run of unmeasured rays is sifted with those rays skipped, while the matcher matches a version of it where they were filled by interpolation across the gap. Measured on a fixture with a forty ray hole, the filled ranges differ from the truth by up to 1.11 m across the chord.

This is the division of labour the design asks for: reduction and gap filling belong to the matcher, and the metric counts only rays that are valid on both sides. It is recorded because the sentence "the same reduced scan feeds all three stages" is true of the vector handed over and not of what the matcher then does with it, and somebody will eventually read the first and assume the second.

### A measurement entirely at maximum range is refused coherently but not informatively

Where the measured scan itself reports nothing but its maximum range, every candidate scores infinity, so the survivors of the sift are arbitrary and the matcher then refuses each of them on its own first cast. The run is refused, correctly, but under the least specific reason available rather than under one saying that the measurement carried no information. A test pins the current behaviour, so making it more informative would be a deliberate change rather than an accidental one.

### One refusal is not reachable through an occupancy grid, and the guard is kept anyway

`no_scorable_pose` is raised where every corrected pose rescores to infinity, meaning the winner's own cast into the map met nothing. That cannot happen with an occupancy grid. The matcher tests every step it takes against the grid's own notion of free space, so a step that would leave free space or leave the grid triggers a recovery instead of being accepted, and a cast from free space in a bounded room always meets a wall. Measured over eighty attempts on both fixture grids, with five poses hugging the border and four measurements chosen to send the matcher outward, with the translation bound both on and off: seventy seven answered, three were refused, none landed off free space, and the furthest any pose walked was 1.554 m.

The guard is kept because a localiser is constructed around a grid reference rather than around an occupancy grid by name, so a caller may hand in something else, and the guard is what stops such a reference producing a confident answer from a cast that met nothing. It is covered by a test that supplies exactly that, with the artificiality stated in the test so nobody removes the guard as unreachable.

### Hypotheses are drawn at cell centres

A drawn position is the centre of a free cell. On a map of twenty centimetre cells that is up to fourteen centimetres of error in the pose the refinement starts from, which the refinement has to absorb. Drawing uniformly within the chosen cell instead is a three line change that does not touch the interface. It has now been measured and it is not needed: holding the build and the maps fixed and changing only the draw, the cell centre set was equal or slightly better in all seven pairs measured, and jittering each hypothesis inside its own cell moves the distance to the nearest hypothesis by a few millimetres either way. At the shipped density the spacing between hypotheses is about 0.26 m, which dwarfs a quantisation of at most 0.141 m.

## What the sensor has to be

### Sensors that do not see all the way round are unsupported

The ROS 1 version carried two refinement paths: the scan matcher for a panoramic sensor and an ICP path for anything narrower. Only the matcher remains, so only a panoramic sensor is supported. This is inherited from `fsm_localisation`, which records the same gap.

A scan whose first ray does not sit half a turn behind the sensor's heading, or whose rays do not span exactly one turn, is refused at the point of use rather than answered wrongly. Refusing is the honest form of the limitation: such a scan is perfectly legal, and reading it as though it began where it did not would return a confident answer rotated by half a turn.

## The node

### The map's own frame is warned about rather than checked

The answer's coordinates are in whatever frame the grid's origin is expressed in, and its label is `fixed_frame_id`. Nothing ties the two together, so publishing a map in one frame and configuring another gives an answer that is numerically right and labelled wrong. The node logs a warning naming both frames and does not refuse, because a mismatch is a configuration smell rather than an impossibility, and because refusing would break a working setup that happens to name its frames differently. `fsm_localisation` ignores the map's frame too.

### The published covariance is a seed and not an estimate

`initial_cov_xx`, `initial_cov_yy` and `initial_cov_aa` are written onto the answer exactly as they were configured. Nothing measures them and nothing about a run changes them, so a run that recovered the pose to a millimetre and a run that scraped in at half a metre are published with the same three numbers. They are there because a consumer such as amcl wants a covariance to be seeded with, and the shipped values are the ones the ROS 1 package shipped.

The material a real estimate could be built from does exist inside a run: the winning pose's own score, and the scores and spread of the survivors the sift handed to the refinement. Nothing turns any of it into a covariance, and doing so would be a change of what this package promises rather than a tidying of it. Until then, a consumer weighing this covariance against its own is weighing it against a constant, and the readme says so where it documents the three settings.

### No setting can be changed while the node runs

Every parameter is read once, at construction, and carries a read-only descriptor. A runtime set is therefore refused rather than accepted and silently ignored, which is the honest of the two behaviours, but it does mean there is no dynamic reconfiguration at all. Relaunching is the way to change a setting.

No setting carries a descriptor **range**, deliberately. A range makes the middleware reject an out-of-range override with its own message, which names the setting but not the value it was given, where every refusal in this package names both. The wording belongs to the two validators.

### Four refusals go beyond what the interface documentation lists

Each is worded like the rest and each follows `fsm_localisation`. A negative value for any whole-number setting, since a negative integer read as unsigned becomes an enormous positive one that no validator can tell from a deliberate setting. A non-finite or negative `tf_lookup_timeout`, which would otherwise reach a duration constructor undefined. An empty `map_topic`, `output_pose_topic` or `scan_topic`. And a frame id that is empty or begins with a slash, which closes the ROS 1 version's hardcoded slashed frame at the door rather than downstream.

### The caster method name and the scan reliability name are the node's alone to refuse

Neither the algorithm's validator nor the matcher's has a rule for either. The node refuses an unknown name when it parses it, in the same words the other refusals use. This is a seam rather than a hole, and it is recorded so that nobody moves the check into a validator that has no business owning it.

## What the tests do not cover

### Three of the ROS 1 defects have no counterpart to test

The version this port replaces carried at least fifteen defects. Thirteen were found by reading it against its own documentation, of which ten are guarded by a named test that would catch a regression. Two more were found later, while its hypothesis dispersal was being measured: a dropped map origin orientation, and a grid cast that never subtracts the map origin at all. Both are demonstrated in the sibling repository and neither has a counterpart here, the arithmetic they live in having gone with the code. The other three were removed by deleting the code that held them, so there is nothing left to test: a variable length array on the stack in the PNG conversion, a read before the start of an empty region list in the scan interpolation, and two allocations that were never freed. All three went with the PNG round trip, the interpolation routine and the vendored particle filter respectively.

### The linter pass is thinner than its count suggests

Six linters run and all six pass, but of them only five examine anything of this package and one skips itself. `cppcheck` skips itself entirely on the current image for upstream performance reasons, all fifteen of its cases, and skips `fsm_localisation` identically. `uncrustify` and `cpplint` are not among the six at all: they are switched off exactly as `fsm_localisation` switches them off, which is a gap rather than a decision: the house style is enforced at review rather than by a tool.

### Concurrency was established rather than assumed, and the arrangement rests on a compile-time guarantee

The sift is split by hypothesis index, one thread per contiguous chunk, each writing only into its own slice, with the ranking computed afterwards from the whole vector, so the answer does not depend on the thread count. One map serves every thread rather than one caster being built per thread. That is safe because all three casters in `fsm_localisation` are read only with their caches compiled out, held that way by a static assertion in that package. If that assertion is ever relaxed, this arrangement stops being safe and one caster per thread becomes necessary.

## Build, packaging and linting

### The deterministic transform option cannot bind the matcher from this package alone

`CBGL_DETERMINISTIC_FFT` makes transform plans chosen by heuristic rather than by timing, so that two runs agree. The flag is read by a header-only file, and `fsm_localisation`'s own library was compiled with `fsm_localisation`'s own setting, so which definition survives linking is the linker's choice. Setting the option on `cbgl` alone is therefore not enough: both packages must be built with their respective options in the same invocation. The readme says so where it documents the options. This is inherited from how `fsm_localisation` arranges it and was out of scope to change here.

### The exported CMake extras file earns nothing today

`cmake/cbgl-extras.cmake` guards the creation of an imported FFTW target, and that target already exists by the time the file runs, because `fsm_localisation`'s own extras file runs first through the exported dependency. The file is kept because the layout names it and because it makes the requirement a stated one rather than an accident of include order, and it costs a consumer nothing. It could go.

### A configuration notice from CGAL cannot be removed

CGAL's own configuration hook emits a notice about the build type being empty. Removing it would take either setting a build type, which would fight the explicit optimisation flags this package sets, or setting CGAL's own opt-out variable, which `fsm_localisation` does not do. Left alone, and identical to what that package emits.

### Neither this repository nor its proof tree carries a pre-commit configuration

Neither does the rest of this family. There is therefore nothing for a pre-commit pass to run, and the absence is recorded here rather than being skipped silently at every commit. Adding one would be a change to how the family works rather than to this package.

## What the comparison against the ROS 1 build settled, and the one thing it did not

That comparison lives in a sibling repository, `cbgl-tests`, laid out beside this one so that recordings, containers and build outputs never weigh down a clone of the package. **It has now been run.** Both halves of it, as each build ships and with both handed one hypothesis set, over seven scenario and resolution pairs, 5600 trials, against the ROS 1 build at `fb9dab1` of 15 May 2024, which was then the last state whose caster ran. The tip has since been repaired and runs too, so that phrasing no longer identifies a state on its own; the commit does. Its figures are in that repository's own results document. Two of the three questions below are answered by it, and so is the cell centre question higher up this file.

### The wider translation bound is right and stays at 1.2 m

`xy_bound` is shipped at 1.2 m against the matcher's own default of 0.2 m, because a survivor of the sift stands at a cell centre and can be a cell or more from the truth. Measured over the 1005 trials that met the bar: the answer sits a median of 0.124 m from the nearest hypothesis it was handed, ninetieth percentile 0.224 m, worst 0.370 m, and 165 of the 1005 are beyond 0.2 m. So a 0.2 m bound would clip about one trial in six. The ROS 1 build needs the same reach, 0.132 m median and 0.364 m worst, so the requirement comes from the shipped hypothesis density rather than from anything this port chose. The refinement is not wandering: all 1005 landed inside the bar.

### Refining against the map costs less per run, by three to eleven times

The ROS 1 refinement cast once per survivor and this one casts at every pose it tries, so a refinement is not obviously cheaper per survivor. The whole run is nonetheless faster: 3.30 to 5.16 times on raw wall clock against the ROS 1 build's matcher path, and 6.09 to 10.35 times against the ICP path it ships by default. Those ratios are an upper bound, the two builds being unable to measure the same interval because the ROS 1 service call disperses and returns rather than localising; crediting it its own self report gives a floor of 3.08 to 5.00. The earlier finding within this build still holds, that `top_k_caers` governs execution time far more than the map's size does.

### Whether the concurrency holds up on a real building is still open

The hypothesis count is proportional to the map's free area, so a large building at the shipped density draws hundreds of thousands of candidates. The largest map the comparison ran on is a rebuilt real environment of about 202 square metres of free space, which draws about three thousand. That is two orders of magnitude short of the case this entry is about, so nothing here answers it and nothing in the readme should be read as claiming otherwise. The mitigating finding is that the sift, which is the part that grows with the map, is the cheap half.
