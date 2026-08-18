# Copyright 2026 Alexandros Filotheou
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
One localisation, end to end: where the answer lands and what it is labelled.

A map and one scan are published from the fixtures, the service is called, and
the pose that comes back is held to the pose the scan was cast from. The scan is
a cast from the room's own polygons rather than from the grid, so the grid is the
only approximation in the chain, which is the honest case and the harder one.

**What the answer is held to.** Four cells of position, at five centimetre cells.
One cell is what the map itself can support, since a ray stops at a cell. The ray
marching caster can stop up to sqrt(2) cells past a wall's face on top of that,
and the hypothesis the answer descends from was drawn at the centre of a cell, so
half a cell more. Four is the three with a cell to spare. The heading gets the
same four cells expressed as an angle, computed from the ranges the fixture scan
actually carries, because a heading wrong by dt moves what the sensor would see
at range r sideways by r * dt.

**The answer places the robot, and the sensor is not the robot.** The fixture's
truth is a sensor pose, since the sensor is what casts. The transform tree
carries where the sensor sits on the robot, and that placement is further from
the origin than the tolerance above, so a node that published the sensor's pose
where the robot's was asked for fails every case here rather than passing by
coincidence. The tree is asked for the robot frame against the frame the scan
message itself names, which is a name no parameter of this node mentions.

**One run, many assertions.** The localisation happens once, before any case
runs, and the cases read what it produced. That keeps the file to two runs and,
more usefully, means the window in which nothing was broadcast is a window in
which only the node that must not broadcast had run at all.

**A second node broadcasts.** The transform is intrusive, so it is off by
default and the answer must be right without it. That the answer is right with it
off is exactly what the Kinetic build got wrong: it looked the sensor's placement
up only when broadcasting, and composed every cast with an uninitialised
transform otherwise.
"""

import unittest

from cbgl_test_support import (
    compose,
    GLOBAL_FRAME,
    Harness,
    heading_tolerance,
    keep_alive,
    node_parameters,
    ODOM_FRAME,
    read_grid,
    read_scans,
    read_truth,
    ROBOT_FRAME,
    robot_from_sensor,
    ROBOT_IN_ODOMETRY,
    SCAN_STAMP,
    shifted,
    world_from_room,
    wrap,
    yaw_of,
)
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy

AGEING_MAP_TOPIC = '/ageing/map'
AGEING_ODOM_FRAME = 'ageing_odometry_frame'
AGEING_POSE_TOPIC = '/ageing/pose'
AGEING_SCAN_TOPIC = '/ageing/scan'

BROADCASTING_POSE_TOPIC = '/broadcasting/pose'

BUFFER_HISTORY = 10.0

CELLS = 4.0

COVARIANCE_XX = 0.11
COVARIANCE_YY = 0.22
COVARIANCE_AA = 0.33

ROBOT_IN_AGEING_ODOMETRY = (-0.90, 1.40, 0.25)
ROBOT_IN_AGEING_ODOMETRY_LATER = (6.00, -4.00, -1.20)


@pytest.mark.launch_test
def generate_test_description():
    """Launch one node that broadcasts a transform and one that does not."""
    quiet = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name='cbgl',
        output='screen',
        parameters=[node_parameters(initial_cov_xx=COVARIANCE_XX,
                                    initial_cov_yy=COVARIANCE_YY,
                                    initial_cov_aa=COVARIANCE_AA)],
    )
    broadcasting = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name='cbgl_broadcasting',
        output='screen',
        parameters=[node_parameters(
            tf_broadcast=True, output_pose_topic=BROADCASTING_POSE_TOPIC)],
    )
    ageing = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name='cbgl_ageing',
        output='screen',
        parameters=[node_parameters(
            tf_broadcast=True,
            odom_frame_id=AGEING_ODOM_FRAME,
            map_topic=AGEING_MAP_TOPIC,
            scan_topic=AGEING_SCAN_TOPIC,
            output_pose_topic=AGEING_POSE_TOPIC,
            dl=150.0, top_k_caers=500, num_threads=1)],
    )
    return (
        launch.LaunchDescription(
            [quiet, broadcasting, ageing, keep_alive(),
             launch_testing.actions.ReadyToTest()]),
        {'cbgl': quiet, 'cbgl_broadcasting': broadcasting,
         'cbgl_ageing': ageing},
    )


class TestNodeBehaviour(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.grid = read_grid()
        scans = read_scans()
        truth = read_truth()

        assert len(scans) == len(truth), \
            'the scan and truth fixtures disagree on how many poses they carry'

        cls.scan = scans[0]
        cls.truth = world_from_room(cls.grid, truth[0])
        cls.expected = robot_from_sensor(cls.truth)

        cls.harness = Harness()
        cls.broadcasting = Harness(node_name='cbgl_broadcasting',
                                   output_pose_topic=BROADCASTING_POSE_TOPIC)
        cls.ageing = Harness(node_name='cbgl_ageing',
                             output_pose_topic=AGEING_POSE_TOPIC,
                             map_topic=AGEING_MAP_TOPIC,
                             scan_topic=AGEING_SCAN_TOPIC)

        cls.ageing.wait_for_connection()
        cls.harness.wait_for_connection(subscribers=2)
        cls.harness.publish_sensor_placement()
        cls.harness.publish_odometry_placement()
        cls.harness.publish_map(cls.grid)
        cls.harness.spin(1.0)
        cls.harness.publish_scan(cls.scan)
        cls.harness.spin(0.5)

        cls.response = cls.harness.call()
        cls.harness.spin(1.0)

        cls.poses = list(cls.harness.poses)
        cls.transforms = list(cls.harness.transforms)

    @classmethod
    def tearDownClass(cls):
        cls.ageing.destroy_node()
        cls.broadcasting.destroy_node()
        cls.harness.destroy_node()
        rclpy.shutdown()

    def answer(self):
        """Return the one pose the localisation of setUpClass published."""
        self.assertTrue(self.response.success, self.response.message)
        self.assertEqual(len(self.poses), 1,
                         'the call published %d poses where one was asked for'
                         % len(self.poses))
        return self.poses[0]

    def test_1_the_answer_places_the_robot_where_the_scan_was_cast_from(self):
        pose = self.answer().pose.pose
        tolerance = CELLS * self.grid.resolution

        self.assertAlmostEqual(
            pose.position.x, self.expected[0], delta=tolerance,
            msg='the scan was cast from a sensor at (%.4f, %.4f, %.4f), which '
                'puts the robot at (%.4f, %.4f, %.4f) through the placement in '
                'the transform tree. The node placed it at (%.4f, %.4f, %.4f). '
                'The bound is %g cells of %g metres'
                % (self.truth + self.expected
                   + (pose.position.x, pose.position.y, yaw_of(pose.orientation))
                   + (CELLS, self.grid.resolution)))
        self.assertAlmostEqual(pose.position.y, self.expected[1],
                               delta=tolerance)

        self.assertAlmostEqual(
            wrap(yaw_of(pose.orientation) - self.expected[2]), 0.0,
            delta=heading_tolerance(self.scan, self.grid.resolution, CELLS),
            msg='the robot faces %.4f radians and the node reported %.4f. The '
                'bound is %g cells seen at the mean range this scan carries'
                % (self.expected[2], yaw_of(pose.orientation), CELLS))

    def test_2_the_answer_is_stamped_from_the_scan_that_produced_it(self):
        stamp = self.answer().header.stamp

        self.assertEqual(
            (stamp.sec, stamp.nanosec), SCAN_STAMP,
            'the answer is stamped from the clock at the moment it was '
            'published rather than from the scan it was computed from, so a '
            'consumer cannot tell what the answer is an answer about')

    def test_3_the_answer_is_expressed_in_the_frame_that_was_configured(self):
        frame_id = self.answer().header.frame_id

        self.assertEqual(
            frame_id, GLOBAL_FRAME,
            'the answer is labelled %r where the configured global frame is '
            '%r. A frame written into the source ignores the setting, and a '
            'leading slash is rejected outright by tf2' % (frame_id,
                                                           GLOBAL_FRAME))
        self.assertFalse(frame_id.startswith('/'), frame_id)

    def test_4_the_answer_carries_the_covariance_it_was_configured_with(self):
        covariance = self.answer().pose.covariance

        self.assertAlmostEqual(covariance[0], COVARIANCE_XX)
        self.assertAlmostEqual(covariance[7], COVARIANCE_YY)
        self.assertAlmostEqual(covariance[35], COVARIANCE_AA)

    def test_5_nothing_is_broadcast_where_broadcasting_was_not_asked_for(self):
        self.assertEqual(
            self.transforms, [],
            'a transform was broadcast into a tree the node was never asked '
            'to touch')

    def test_6_the_broadcast_transform_carries_the_answer(self):
        """
        Broadcast a transform that chains onto the odometry frame.

        What makes the transform useful is that composing it with the odometry
        frame's own place in the tree gives the pose the node published, so that
        is what is asserted rather than the numbers it happens to carry.
        """
        self.broadcasting.clear()
        self.broadcasting.publish_scan(self.scan)
        self.broadcasting.spin(0.5)

        response = self.broadcasting.call()
        self.broadcasting.spin(1.0)

        self.assertTrue(response.success, response.message)
        self.assertEqual(len(self.broadcasting.poses), 1)

        broadcast = [transform
                     for message in self.broadcasting.transforms
                     for transform in message.transforms
                     if (transform.header.frame_id == GLOBAL_FRAME
                         and transform.child_frame_id == ODOM_FRAME)]

        self.assertEqual(
            len(broadcast), 1,
            'the node broadcast %d transforms from %r to %r, where one was '
            'asked for. What it did broadcast was %r'
            % (len(broadcast), GLOBAL_FRAME, ODOM_FRAME,
               [(t.header.frame_id, t.child_frame_id)
                for message in self.broadcasting.transforms
                for t in message.transforms]))

        stamp = broadcast[0].header.stamp
        self.assertEqual((stamp.sec, stamp.nanosec), SCAN_STAMP)

        placed = compose(
            (broadcast[0].transform.translation.x,
             broadcast[0].transform.translation.y,
             yaw_of(broadcast[0].transform.rotation)),
            ROBOT_IN_ODOMETRY)

        pose = self.broadcasting.poses[0].pose.pose

        self.assertAlmostEqual(placed[0], pose.position.x, delta=1e-6)
        self.assertAlmostEqual(placed[1], pose.position.y, delta=1e-6)
        self.assertAlmostEqual(wrap(placed[2] - yaw_of(pose.orientation)), 0.0,
                               delta=1e-6)

    def test_7_the_broadcast_outlives_the_history_the_tree_keeps(self):
        """
        Broadcast from placements read before the search rather than after it.

        **What this guards.** Both placements the answer needs are read from the
        transform tree when the run begins. Reading the odometry one afterwards
        instead looks perfectly correct and works on every small map: the run
        ends a third of a second later and the tree still remembers the stamp the
        scan carried. On a real building at the shipped density a run draws
        hundreds of thousands of candidates and takes far longer than the ten
        seconds of history the tree keeps, so the stamp asked for at the end has
        been forgotten, the lookup fails, and nothing is broadcast at all. The
        shipped configuration turns broadcasting on, so that is the intended
        target quietly losing the transform it was configured to publish.

        **How the tree is made to forget, cheaply.** Not by making a run last
        longer than the history, which would cost this suite forty seconds of
        waiting. The history is bounded relative to the newest transform the tree
        holds, not to the clock, so publishing odometry stamped twenty seconds
        past the scan while the run is still going evicts the scan's own stamp
        just as surely as waiting would. What is left is a tree that could answer
        for the scan's stamp when the run began and cannot by the time it ends.

        **The odometry placement alone is aged.** It is published here on the
        moving transform topic, which is where a robot publishes it and where it
        expires. The sensor's placement stays where the rest of this file puts
        it, on the static topic, because only one of the two is being aged
        deliberately and a case that aged both could not say which read was too
        late.

        **This case cannot fail if that publication is moved onto the static
        topic**, since a static transform never expires. Anybody tidying it that
        way has disarmed the case rather than simplified it.

        The transform topic is written to sparingly, five messages in all. A
        stream of them fills the subscription queue and pushes the node's own
        single broadcast out of it, which looks exactly like the node not having
        broadcast anything.
        """
        self.ageing.clear()
        self.ageing.publish_sensor_placement()
        self.ageing.publish_map(self.grid)
        self.ageing.spin(1.0)
        self.ageing.publish_scan(self.scan)

        for offset in (-0.2, 0.0, 0.2):
            self.ageing.publish_dynamic_transform(
                AGEING_ODOM_FRAME, ROBOT_FRAME, ROBOT_IN_AGEING_ODOMETRY,
                shifted(SCAN_STAMP, offset))

        self.ageing.spin(0.5)

        def age_the_tree_past_the_scan():
            for offset in (2.0 * BUFFER_HISTORY, 2.0 * BUFFER_HISTORY + 0.2):
                self.ageing.publish_dynamic_transform(
                    AGEING_ODOM_FRAME, ROBOT_FRAME,
                    ROBOT_IN_AGEING_ODOMETRY_LATER,
                    shifted(SCAN_STAMP, offset))

        outcome = self.ageing.call_while(age_the_tree_past_the_scan, delay=0.5)
        self.ageing.spin(1.0)

        self.assertTrue(outcome.success, outcome.message)
        self.assertEqual(len(self.ageing.poses), 1,
                         'the run published %d poses where one was asked for'
                         % len(self.ageing.poses))

        broadcast = [transform
                     for message in self.ageing.transforms
                     for transform in message.transforms
                     if (transform.header.frame_id == GLOBAL_FRAME
                         and transform.child_frame_id == AGEING_ODOM_FRAME)]

        self.assertEqual(
            len(broadcast), 1,
            'the node broadcast %d transforms from %r to %r where one was '
            'asked for. A run whose placements are read after the search asks '
            'the tree for a stamp it has stopped holding, and broadcasts '
            'nothing at all'
            % (len(broadcast), GLOBAL_FRAME, AGEING_ODOM_FRAME))

        stamp = broadcast[0].header.stamp
        self.assertEqual((stamp.sec, stamp.nanosec), SCAN_STAMP)

        placed = compose(
            (broadcast[0].transform.translation.x,
             broadcast[0].transform.translation.y,
             yaw_of(broadcast[0].transform.rotation)),
            ROBOT_IN_AGEING_ODOMETRY)

        pose = self.ageing.poses[0].pose.pose

        self.assertAlmostEqual(
            placed[0], pose.position.x, delta=1e-6,
            msg='the broadcast transform does not compose with where the robot '
                'stood in the odometry frame to give the answer that was '
                'published, so it was built from a placement other than the one '
                'the tree held at the scan')
        self.assertAlmostEqual(placed[1], pose.position.y, delta=1e-6)
        self.assertAlmostEqual(wrap(placed[2] - yaw_of(pose.orientation)), 0.0,
                               delta=1e-6)


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_the_nodes_exited_cleanly(self, proc_info, cbgl, cbgl_ageing,
                                      cbgl_broadcasting):
        for process in (cbgl, cbgl_ageing, cbgl_broadcasting):
            launch_testing.asserts.assertExitCodes(proc_info, process=process)
