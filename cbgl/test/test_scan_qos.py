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
The scan subscription's reliability and depth, at values it does not ship with.

A sensor driver commonly publishes best effort, and a subscription that does not
match its publisher receives nothing whatever. The node therefore offers the
reliability as a setting, and this file is the only place where its other
permitted value, and a queue depth other than the one it ships with, are asked
for by a launch and then found on the subscription the node really created.

**Why a best effort publisher is what asks the question.** A reliable
subscription and a best effort publisher do not match, so a node that declared
the setting and then ignored it would receive no scan at all from the harness
here and would refuse the run for want of one. The other way round proves
nothing: a best effort subscription happily takes a reliable publisher's
messages, so a harness publishing reliably would pass whether the setting
arrived or not.

**What can be asked of the queue depth, and what cannot.** One scan is all a run
uses and the node keeps the newest, so no depth changes the answer, and there is
nothing behavioural to assert. What is asserted is that the number the launch
asked for is the number the subscription carries, which is what catches a setting
that was declared and then dropped on the way to the subscription.

**Nothing is driven until the subscriptions have been looked at.** A best effort
publisher and a reliable subscription never match, so a node that dropped the
setting leaves the harness waiting for a subscriber that will never appear. The
two cases that read the subscriptions therefore run before anything is published,
so that a dropped setting is reported as the wrong reliability on the
subscription rather than as a wait that timed out.

**The map subscription is the control.** It is left alone by both settings and
has to stay latched, reliable and one deep, so that a change reaching further
than it was meant to shows up here rather than in somebody's map server.

**What the answer is held to.** Four cells of position at five centimetre cells,
and the same four cells expressed as an angle at the ranges the fixture scan
carries, which is the bound the other end to end cases use and for the reasons
they give: a cell for what the map can support, the ray marching caster's
overshoot past a wall's face on top of it, half a cell for the centre of the cell
the hypothesis was drawn at, and a cell of slack over the three. The pose is the
robot's rather than the sensor's, through the placement the transform tree
carries.
"""

import time
import unittest

from cbgl_test_support import (
    Harness,
    heading_tolerance,
    keep_alive,
    node_parameters,
    read_grid,
    read_scans,
    read_truth,
    robot_from_sensor,
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
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

CELLS = 4.0

SCAN_QOS_DEPTH = 5
SCAN_QOS_RELIABILITY = 'best_effort'


@pytest.mark.launch_test
def generate_test_description():
    """Launch one node subscribed to scans best effort and five deep."""
    node = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name='cbgl',
        output='screen',
        parameters=[node_parameters(
            scan_qos_reliability=SCAN_QOS_RELIABILITY,
            scan_qos_depth=SCAN_QOS_DEPTH)],
    )
    return (
        launch.LaunchDescription(
            [node, keep_alive(), launch_testing.actions.ReadyToTest()]),
        {'cbgl': node},
    )


class TestScanQualityOfService(unittest.TestCase):

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

        cls.harness = Harness(scan_reliability=SCAN_QOS_RELIABILITY,
                              scan_depth=SCAN_QOS_DEPTH)

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    def endpoint_on(self, topic, timeout=30.0):
        """Return what the node's subscription to a topic was created with."""
        deadline = time.monotonic() + timeout

        while True:
            found = self.harness.get_subscriptions_info_by_topic(topic)

            if found:
                return found[0]

            self.assertLess(time.monotonic(), deadline,
                            'nothing subscribes to %s' % topic)
            rclpy.spin_once(self.harness, timeout_sec=0.1)

    def test_1_the_scan_is_subscribed_the_way_the_launch_asked_for(self):
        scan_subscription = self.endpoint_on('/scan')

        self.assertEqual(
            scan_subscription.qos_profile.reliability,
            ReliabilityPolicy.BEST_EFFORT,
            'the launch asked for %r and the subscription is %r. A sensor '
            'driver publishing best effort would be talking to a subscription '
            'that cannot hear it, and the node would wait for a scan that is '
            'already being published'
            % (SCAN_QOS_RELIABILITY, scan_subscription.qos_profile.reliability))

        self.assertEqual(
            scan_subscription.qos_profile.depth, SCAN_QOS_DEPTH,
            'the launch asked for a queue %d deep and the subscription carries '
            '%d, so the setting was declared and then dropped on its way to '
            'the subscription'
            % (SCAN_QOS_DEPTH, scan_subscription.qos_profile.depth))

    def test_2_the_map_is_subscribed_as_it_always_is(self):
        map_subscription = self.endpoint_on('/map')

        self.assertEqual(map_subscription.qos_profile.durability,
                         DurabilityPolicy.TRANSIENT_LOCAL,
                         'the scan settings reached the map subscription and '
                         'cost it the durability a map server needs it to have')
        self.assertEqual(map_subscription.qos_profile.reliability,
                         ReliabilityPolicy.RELIABLE)
        self.assertEqual(map_subscription.qos_profile.depth, 1)

    def test_3_a_best_effort_scan_is_received_and_localised_from(self):
        self.harness.wait_for_connection()
        self.harness.publish_sensor_placement()
        self.harness.publish_map(self.grid)
        self.harness.spin(1.0)

        self.harness.clear()
        self.harness.publish_scan(self.scan)
        self.harness.spin(0.5)

        response = self.harness.call()

        self.assertTrue(
            response.success,
            'the run was refused: %s. The scan was published best effort, '
            'which a subscription still asking for reliable delivery never '
            'receives, so a refusal for want of a scan means the setting the '
            'launch gave never reached the subscription' % response.message)

        self.harness.spin(1.0)

        self.assertEqual(len(self.harness.poses), 1,
                         'the call published %d poses where one was asked for'
                         % len(self.harness.poses))

        pose = self.harness.poses[0].pose.pose
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
            delta=heading_tolerance(self.scan, self.grid.resolution,
                                    CELLS),
            msg='the robot faces %.4f radians and the node reported %.4f. The '
                'bound is %g cells seen at the mean range this scan carries'
                % (self.expected[2], yaw_of(pose.orientation),
                   CELLS))


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_the_node_exited_cleanly(self, proc_info, cbgl):
        launch_testing.asserts.assertExitCodes(proc_info, process=cbgl)
