^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cbgl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Leaves a gap a gap in a scan already ``size_scan`` rays long whose rays lie
  off the angles they are compared at. Such a scan was filled across its gaps
  before it was read at those angles, so the sift compared ranges the sensor
  never measured. The same scan with its rays on those angles is sifted with
  its gaps skipped. A ray read from a missing reading, wholly or in part, is
  now missing too, so a sensor a little off those angles treats a gap as one
  on them does. A gap of m rays between two measured rays leaves at most
  (m + 1) r rays missing, rounded up, where r is the sensor's spacing over the
  compared spacing. The gap thus grows by one ray at most where the two
  spacings agree, and by two at most where they do not. The matcher still
  fills gaps itself. A scan reduced to fewer rays is still filled first. In a
  test of 360 rays with a gap of forty, the scan half a ray off its angles was
  sifted over 360 rays and is now sifted over 319, where the scan on its
  angles is sifted over 320. 500 rays 2 pi / 999.6 apart are compared
  2 pi / 1000 apart, and a gap of forty of them leaves 42 rays missing at 16
  of a thousand first angles across one ray.
* Reads a scan of a whole turn between its last ray and its first where a ray
  is compared between them. A ray compared beyond the sensor's first or last
  ray took that ray's range. That suits a sensor that does not see all the way
  round, and not one that does, whose last and first rays are a ray apart like
  any other two. 360 rays a degree apart from -179.5 degrees compared ray 0 at
  -180 degrees and gave it the range taken at -179.5. It now reads the mean of
  the rays taken at 179.5 and -179.5. A whole turn whose rays lie on the
  angles they are compared at, as one from minus pi or from plus pi does, is
  read as before.
* Refuses a scan whose ``angle_min`` or ``angle_increment`` is not finite,
  naming the field and the value it carried. The node's test of a scan's
  angles asked whether each was out by more than one ray, and every comparison
  with not a number is false. A first angle of not a number passed and was
  turned into an index of the scan where it is sampled, which is undefined. In
  the package's test room such a run was refused for a scan carrying no usable
  range. A first angle of infinity passed as well and was answered with a
  pose. An infinite spacing passed too, since it allows an infinite slack, and
  the matcher then refused every survivor for an aperture no sensor can cover.
  Each of the three is now refused by the node, before a hypothesis is drawn.
* Tells the matcher where the first ray of the scan it refines lies, which
  ``fsm_localisation::Corrector::correct`` now requires, with no default. The
  scan it is given has been read at the angles the window compares it at, so
  its first ray lies at ``FSM::windowStartOf`` of its ray count and the
  aperture, not at the sensor's ``angle_min``, and that is the angle passed.
  This needs an ``fsm_localisation`` whose matcher takes the argument, and an
  older ``cbgl`` does not build against one that does.
* Treats a reading below the scan's own ``range_min`` or above its own
  ``range_max`` as missing, exactly as it treats one that is not a number. By
  the ROS convention neither is a distance to anything, and both were read as
  returns wherever they were finite and above nought: a wall touching the
  sensor, or a wall farther away than the sensor can see. The sensor of a real
  robot this package has been run against reports 0.001 m against a
  ``range_min`` of 0.023 m in one and a half per cent of its readings. In the
  package's test room, a scan with a sector of each kind was answered 2.030 m
  and 139.9 degrees from the same scan with those sectors missing, and is now
  answered 0.000 m and 0.000 degrees from it in each of three runs. A reading
  equal to either limit is still a return. The node makes such readings
  missing before the scan reaches ``Localiser::localise``, which still takes
  no minimum range.
* Reads a scan reduced to fewer rays at the angles its reduced rays are
  compared at. The reduction laid the scan over a whole turn whatever the
  sensor covered, so for a sensor that does not see all the way round each
  reduced ray held a range taken somewhere other than where it was compared.
  Reducing 1073 rays a quarter of a degree apart to 269 compared reduced ray 0
  at -134.127 degrees and gave it the range taken at -134.000. Each reduced ray
  is now the scan interpolated between the two sensor rays either side of the
  angle it is compared at, and one compared beyond the sensor's first or last
  ray takes that ray's range. In the package's test room, averaged over three
  poses, the answer leaned 0.153 degrees before and leans 0.018 now.
  ``Localiser::localise`` takes the scan's first angle for this, with no
  default, and the node passes the scan's own ``angle_min``. A scan already
  ``size_scan`` rays long, as every scan is with ``size_scan`` at ``0``, is
  read the same way where its rays do not lie on the angles they are compared
  at. A centred scan of 1072 rays a quarter of a degree apart has every ray
  compared half a ray, 0.125 degrees, clockwise of where it was taken. In the
  same room it leaned 0.140 degrees and leans 0.000 now. Where the rays do lie
  on those angles, to within what rounding the angles to 32 bit floats moves
  them by, the scan is read as before, missing readings and all. The 1073 rays
  above lie there, as does every whole turn from minus pi or from plus pi.
* Lays the optional map scan over the scan the sensor reported, ray for ray.
  It was cast over a whole turn from half a turn behind the answer's heading,
  at the ray count the search was reduced onto, and published under the scan's
  own first angle in the scan's own frame. For a sensor that does not see all
  the way round it was drawn turned and stretched, for one mounted upside down
  it was drawn mirrored, and with ``size_scan`` set it carried a different
  number of rays from the scan it was laid over. It now carries the scan's own
  first angle, spacing and ray count, each of its rays is cast along the
  direction of the sensor's own ray, and for a sensor mounted upside down the
  rays are taken in the order that sensor reports them. In the package's test
  room the cast disagrees with the scan by 0.024 to 0.025 m a ray on average,
  where before it disagreed by 0.31 m mirrored and 0.19 m turned and stretched.
* Reads a sensor mounted upside down from the transform tree, with nothing to
  set. The sensor's placement on the robot was read as a heading and nothing
  more, so a lidar mounted half a turn about its forward axis was taken for an
  upright one and its scan was read as the mirror image of the room. On the
  recording the published real world experiment was run on, whose lidar is
  mounted that way, none of twenty frames was answered inside half a metre and
  five degrees. The whole rotation is now read. The heading is that of the
  sensor's x axis, and where its z axis points down the ranges are taken in
  reverse order and laid out from that heading less the sum of the scan's first
  and last angles. The sum is nought for a scan symmetric about the sensor's
  heading. It is minus one ray for a panoramic scan beginning at minus pi, where
  reversing the ranges alone would turn the answer by one ray. This is what
  ``laser_z_orientation: downwards`` was for in the ROS 1 build.
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
  than from the wall clock. The answer is published in ``fixed_frame_id`` rather
  than in the hardcoded literal ``/map``, which carried a leading slash tf2
  rejects.
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
* Keeps the ROS 1 names ``fixed_frame_id``, ``base_frame_id`` and
  ``odom_frame_id``, with the same meanings and defaults. A configuration
  carried across keeps them as they are. The sensor's own frame is not a setting
  in either version, being read off the scan message.
* Renames the node from ``cbgl_node`` to ``cbgl`` and defaults its namespace to
  ``/`` rather than to ``robot``, so the service that answered at
  ``/robot/cbgl_node/global_localization`` answers at
  ``/cbgl/global_localisation``. Both are launch arguments.
* Removes the ``vanilla`` value of ``map_scan_method``, which selected a second
  ray tracer that is gone, and which was also the value the Kinetic version fell
  back to when the setting was absent. An unrecognised name is now refused at
  start-up rather than quietly replaced.
* Removes ``laser_z_orientation``. In the ROS 1 build it said which way the
  sensor's z axis pointed. Its shipped value, ``upwards``, compensated for that
  build reading the grid's rows in reverse, which the replacement map handles
  itself along with the origin's translation and its rotation. ``downwards``
  served a sensor mounted upside down, which is read from the transform tree
  instead.
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
* Refuses a scan that is not centred on the sensor's heading, and names the
  field and the value it carried. The algorithm reads the map over one turn
  that begins half a turn behind the sensor's heading. It compares only the
  middle of that turn, as wide as the scan. A legal scan that begins elsewhere
  would otherwise be answered with a rotated pose, in silence. A scan that spans
  more than a full turn, or carries an ``angle_increment`` that is not positive,
  is refused too. A scan narrower than a full turn is accepted.
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
