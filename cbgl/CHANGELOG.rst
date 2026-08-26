^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cbgl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Carries one refinement rather than two. The ROS 1 package shipped Andrea
  Censi's canonical scan matcher selected, and this port carried it as an option
  so that a run here could be compared against results published from that
  build. It is removed. The comparison was made, at a thousand trials per
  configuration on a real map: at three quarters of a turn the panoramic scan
  matcher answers 99.6 per cent of trials inside half a metre and five degrees
  with a median error of 0.75 mm, against the solver's 99.8 per cent and 4.32
  mm, and it is the quicker of the two. At a full turn the scan matcher was
  already ahead. Keeping the solver cost an LGPL v3 dependency with no ROS 2
  release for any distribution, three patches against a commit upstream has not
  moved since 2022, and one of those patches correcting a write past the end of
  a buffer inside it.
* ``refinement_method`` takes ``fsm`` or ``none``. The twenty six settings the
  solver read are gone with it, as is ``COPYING.LESSER``: the LGPL v3
  obligation left with the dependency that brought it.
* ``size_scan`` ships ``0``, every ray the scan carries, rather than ``360``. A
  sensor covering three quarters of a turn carries 270 rays and was refused as
  too short, by a message that named the ray count and never the aperture. The
  aperture was always read from the scan; the ray count was not, and shipping
  the value that needs no deriving is what makes such a sensor work with nothing
  set.
* ``Localiser::localise`` no longer takes the scan's minimum range. It reached
  the solver alone and reaches nothing now. Its aperture argument carries no
  default either, for the reason the aperture repair records: a default of a
  whole turn is what lets a caller with a narrower sensor say nothing and be
  answered wrongly.
* Carried from ROS 1 Kinetic and C++11 to ROS 2 Lyrical and C++23. The published
  Kinetic package is preserved unchanged on the ``kinetic-devel`` branch. This
  version is not a drop-in replacement. The readme lists every changed name and
  every changed behaviour.
* Depends on ``fsm_localisation`` and carries no copy of the scan matcher, of the
  ray casting, or of the occupancy grid type. Where the Kinetic package vendored
  a 3781 line copy of the matcher, a copy of ``range_libc`` with graphics card
  kernels, a PNG codec, a copy of ``gflags``, a benchmark harness, amcl's
  particle filter and amcl's map, all of that is now a package dependency.
* Split into three libraries and one executable, all exported under
  ``export_cbgl``. ``cbgl_core`` is the algorithm and knows nothing of ROS, not
  even a message type: it carries no ``rclcpp`` symbol and a consumer can link it
  with ``rclcpp`` absent from the compiler's include path. ``cbgl_grid`` adds the
  free space of an occupancy grid. ``cbgl_interface`` is the node.
* Removes 33,922 lines across 96 files, one of them a PDF, being everything the
  algorithm stopped using when it took the matcher and the casting as a
  dependency.
* The refinement is the matcher only. The ICP path is gone, and with it ``csm``,
  GSL and the egsl bindings. The Kinetic package shipped ``do_icp: true``, so a
  configuration carried across from it was refining by the path that has gone.
  A sensor that does not see all the way round is supported all the same: the
  entry above records what that took and what it is worth.
* Unlike the Kinetic version, the refinement casts into the map at every pose it
  tries rather than matching each survivor against one scan manufactured at that
  survivor's own pose. This is the largest behavioural difference in the port.
* Corrects the labelling of a rotated cast. Candidate ``k`` of a panoramic cast
  now carries the orientation the rotation actually amounts to, derived from the
  shift applied rather than from ``k`` alone. The Kinetic version shifted by
  ``round(n / da) * k`` samples and labelled the result ``-2 * pi * k / da``: at
  360 rays and 32 orientations that is 11 degrees a step against a label moving
  11.25 degrees a step, so the label stood 7.75 degrees from the truth by the
  last candidate and moved in the opposite direction throughout, and the
  refinement started from a pose whose orientation disagreed with the scan it had
  been chosen for.
* Draws hypothesis positions by choosing a free cell uniformly, so every draw is
  on free space by construction. The Kinetic version computed its sampling bounds
  as ``size * scale / 2 -+ origin`` where it meant ``origin -+ size * scale / 2``.
  The two agree whenever the map message origin is zero, which is where a map
  server usually puts it, so the defect is latent there. They diverge for an
  origin away from zero, and for a map centred on the world origin the interval
  collapses to a single point the rejection loop spins on without returning.
* Disperses sensor poses rather than robot poses. The Kinetic version dispersed
  robot poses over free cells and offset to the sensor afterwards, which can
  place the sensor, which is what casts, on a blocked cell.
* Publishes no pose when a run cannot produce one, and reports why. The Kinetic
  version read its winner from index minus one and published from it. It never
  refused a candidate, scoring every one unconditionally; the index stayed at
  minus one when the survivor list was empty, which ``top_k_caers`` at zero
  produces.
* Bounds the partial sort by the candidate count. The Kinetic version called
  ``nth_element`` past the end of its own index vector and then read past it.
* Bounds every other read by what the vector it reads actually carries. The
  Kinetic version cropped a panoramic cast by a width computed in unsigned
  arithmetic that wrapped whenever the cast was shorter than the measurement,
  compared two range vectors over a length neither was known to have, and read
  the first range of a scan before establishing that there was one.
* Reads the sensor's placement on the robot whether broadcasting is on or off.
  With ``tf_broadcast`` false the Kinetic version never looked it up and composed
  every pose with a default constructed transform whose matrix was uninitialised,
  which made that setting unusable there and is why the default here can be
  false.
* Three further Kinetic defects went with the code that held them rather than
  being corrected: a variable length array on the stack in the PNG conversion,
  a read before the start of an empty region list in the scan interpolation, and
  two allocations that were never freed. The follow-up record says so, since
  there is nothing left for a test to guard.
* Stamps every published message from the scan that produced the answer rather
  than from the wall clock, and publishes in ``global_frame_id`` rather than in
  the hardcoded literal ``/map``, which carried a leading slash tf2 rejects.
* Runs the sift across all cores, splitting by hypothesis index so that the
  ranking and the answer do not depend on the thread count.
* Adds ``std_srvs/srv/Trigger`` in place of ``std_srvs/Empty``, and the service
  now returns when the work is done, carrying the outcome. The Kinetic service
  returned before the work was finished and reported nothing about it. The name
  is node relative and fixed at ``global_localisation``, and
  ``global_localisation_service_name`` is gone.
* Adds ``scan_qos_reliability`` and ``scan_qos_depth``, following
  ``fsm_localisation``, so that a driver publishing best effort can be heard.
* Adds three optional diagnostic topics, each off while its name is empty: the
  whole dispersed set, the survivors of the sift in ranking order, and the scan
  the answer would see cast from the map. These replace ``publish_pose_sets``.
* Renames ``fixed_frame_id`` to ``global_frame_id``, following the family, and
  ``base_frame_id`` to ``robot_frame_id``. The second keeps its meaning and its
  ``base_footprint`` default: it is the frame the published pose places, and a
  configuration carried across has to rename it rather than drop it. The sensor's
  own frame is not a setting in either version, being read off the scan message.
* Renames the node from ``cbgl_node`` to ``cbgl`` and defaults its namespace to
  ``/`` rather than to ``robot``, so the service that answered at
  ``/robot/cbgl_node/global_localization`` answers at
  ``/cbgl/global_localisation``. Both are launch arguments.
* Removes the ``vanilla`` value of ``map_scan_method``, which selected a second
  ray tracer that is gone, and which was also the value the Kinetic version fell
  back to when the setting was absent. An unrecognised name is now refused at
  start-up rather than quietly replaced.
* Removes ``laser_z_orientation``. It existed to compensate for reading the
  grid's rows in reverse, which the replacement map handles itself along with the
  origin's translation and its rotation.
* Removes ``do_undersample_scan`` and ``undersample_rate``. ``size_scan`` does
  this, and does it by resampling rather than by dropping every nth ray.
* Removes ``do_icp``, ``do_fsm`` and ``pkg_name``.
* ``configuration_files/params_cbgl.yaml``, ``params_csm.yaml`` and
  ``params_fsm.yaml`` become one ``config/params.yaml`` keyed by the wildcard
  ``/**:``, so that renaming the node cannot silently drop every setting in it.
  ``launch/cbgl.launch`` becomes ``launch/cbgl.launch.xml``.
* Every parameter is validated at start-up. A value outside its range is refused
  with a sentence naming the setting and the value it was given, and the process
  exits with 1 rather than aborting. Every parameter is read only thereafter, so
  a runtime set is refused rather than accepted and quietly ignored.
* Refuses a scan that is not panoramic, naming the field and the value it
  carried. The map is cast over exactly one turn beginning half a turn behind the
  sensor's heading and the measured scan is read the same way, so a legal scan
  beginning elsewhere would otherwise be answered with a pose rotated by half a
  turn, in silence.
* Carries no arithmetic build option of its own. The two that govern the
  arithmetic and the transform planning belong to ``fsm_localisation`` and this
  package follows whatever that package was built with. The matcher is a header
  this package compiles for itself, so two arithmetics over one header would put
  two definitions of one inline function into a single process, and which
  survived linking would be the linker's choice. ``fsm_localisation`` records
  what it was built with and this package reads it, refusing at configure time a
  ``fsm_localisation`` too old to carry that record.
* Becomes MIT. Dropping amcl's map and particle filter removes the LGPL v2.1
  obligation and dropping the vendored casting removes the Apache 2.0 obligation,
  so ``COPYING.LGPL`` and ``LICENSE.APACHE`` are gone and ``LICENSE.MIT`` becomes
  ``LICENSE``. A binary built from these sources links ``fsm_localisation`` and
  therefore carries the terms of the components inside that package if
  redistributed, which the readme states.
* Removes ``INSTALLATION_DEPRECATED.md``, which described a native Ubuntu 16.04
  installation.
* Adds a test suite of 240 cases: the metric, the dispersal, the rotation
  labelling, the sift, the free space, the localiser end to end, the parameters,
  the node's declarations, and six launch tests each in its own domain. Every
  public function of the two ROS-free libraries is exercised and every refusal is
  reached. The rotation labelling cases were shown to fail against the Kinetic
  formula by 1.7176 m against a tolerance of 0.0005 m.
* Adds ``FOLLOWUP.md``, recording known gaps and deferred work.
