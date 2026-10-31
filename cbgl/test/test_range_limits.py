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
Readings outside the scan's own limits, which are not returns.

A scan carries the nearest and the farthest range its sensor can measure, and
by the ROS convention a reading below range_min or above range_max is not a
distance to anything. The sensor of a real robot this package has been run
against reports 0.001 m against a range_min of 0.023 m in one and a half per
cent of its readings. Read as a return, such a reading is a wall touching the
sensor, and a reading above range_max is a wall farther away than the sensor
can see. The answer then moves to agree with walls that are not there.

**One scan, read two ways.** The fixture scan is published with two sectors of
it replaced: one by a reading below the range_min the scan carries, and one by
a reading above its range_max. The same scan is then published with both
sectors not a number, which is what the node already reads as missing. The
node, the map, the seed and so the hypotheses drawn are the same both times,
so a node that reads a reading outside the limits as missing answers the two
scans alike.

**Where the sectors are.** Each spans 60 degrees, centred 45 degrees either
side of the sensor's heading: the readings below range_min to the right, those
above range_max to the left. They are wide so that either kind alone moves the
answer far, and a node that made one kind missing and kept the other would
still fail here. Measured on the code that reads both kinds as returns, three
runs each and every run alike: the readings below range_min alone, with the
others missing, put the robot 2.0987 m and 148.428 degrees from the answer to
the scan with both missing, and the readings above range_max alone put it
1.8576 m and 100.053 degrees from it. The answer to the scan with both missing
stands 0.0210 m and 0.334 degrees from where the robot does.

**What the two answers are held to.** Each other, to one cell of position and
half a ray of heading. The two scans carry the same readings once the ones
outside the limits are missing, so any bound between the two regimes below
would do. One cell and half the scan's own spacing are far from both. Measured,
three runs of each and every run alike: with both kinds read as returns the
two answers are 2.0302 m and 139.928 degrees apart. With the readings outside
the limits made missing in the node before the search, a change made for the
measurement and not kept, they are 0.0000 m and 0.000 degrees apart, and so
is each kind alone. The bound comes to 0.05 m and 0.5 degrees.
"""

import math
import unittest

from cbgl_test_support import (
    FIRST_RAY_ANGLE,
    Harness,
    keep_alive,
    node_parameters,
    RANGE_MAX,
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

RANGE_MIN = 0.023

ABOVE_RANGE_MAX = 1.5 * RANGE_MAX
BELOW_RANGE_MIN = 0.001

ABOVE_SECTOR = (math.radians(45.0), math.radians(60.0))
BELOW_SECTOR = (math.radians(-45.0), math.radians(60.0))


@pytest.mark.launch_test
def generate_test_description():
    """Launch one node, to be handed one scan read two ways."""
    node = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name='cbgl',
        output='screen',
        parameters=[node_parameters()],
    )

    return (
        launch.LaunchDescription(
            [node, keep_alive(), launch_testing.actions.ReadyToTest()]),
        {'cbgl': node},
    )


def rays_of(sector, size):
    """Return the rays of a panoramic scan that a centre and a width cover."""
    centre, width = sector
    increment = 2.0 * math.pi / size
    first = round((centre - width / 2.0 - FIRST_RAY_ANGLE) / increment)

    return [(first + ray) % size for ray in range(round(width / increment))]


def replaced(ranges, below, above):
    """Return ranges with one sector read as below and the other as above."""
    readings = list(ranges)

    for ray in rays_of(BELOW_SECTOR, len(ranges)):
        readings[ray] = below

    for ray in rays_of(ABOVE_SECTOR, len(ranges)):
        readings[ray] = above

    return readings


class TestRangeLimits(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.grid = read_grid()
        cls.scan = read_scans()[0]
        cls.expected = robot_from_sensor(
            world_from_room(cls.grid, read_truth()[0]))

        cls.harness = Harness()
        cls.harness.wait_for_connection()
        cls.harness.publish_sensor_placement()
        cls.harness.publish_map(cls.grid)
        cls.harness.spin(1.0)

        cls.outcomes = {
            'outside': cls.localise(
                replaced(cls.scan, BELOW_RANGE_MIN, ABOVE_RANGE_MAX)),
            'missing': cls.localise(replaced(cls.scan, math.nan, math.nan)),
        }

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    def answer(self, readings):
        """Return the one robot pose published for a scan, as x, y, yaw."""
        response, poses = self.outcomes[readings]

        self.assertTrue(response.success, response.message)
        self.assertEqual(len(poses), 1,
                         'the call published %d poses where one was asked for'
                         % len(poses))

        pose = poses[0].pose.pose
        return (pose.position.x, pose.position.y, yaw_of(pose.orientation))

    @classmethod
    def localise(cls, ranges):
        """Localise one scan and return the response and the poses published."""
        cls.harness.clear()
        cls.harness.publish_scan(ranges, range_min=RANGE_MIN)
        cls.harness.spin(0.5)

        response = cls.harness.call()
        cls.harness.spin(1.0)

        return response, list(cls.harness.poses)

    def test_readings_outside_the_limits_are_read_as_missing(self):
        outside = self.answer('outside')
        missing = self.answer('missing')

        heading_bound = math.pi / len(self.scan)
        position_bound = self.grid.resolution

        position_difference = math.hypot(outside[0] - missing[0],
                                         outside[1] - missing[1])
        heading_difference = abs(wrap(outside[2] - missing[2]))

        message = (
            'with readings of %g m below its range_min of %g m and of %g m '
            'above its range_max of %g m, the scan put the robot at (%.4f, '
            '%.4f, %.4f). With the same readings missing it put the robot at '
            '(%.4f, %.4f, %.4f), %.4f m and %.3f degrees away. The robot '
            'stands at (%.4f, %.4f, %.4f). The bound is %.4f m, one cell, and '
            '%.3f degrees, half a ray. A reading outside the limits the scan '
            'carries is not a return, and was read as one'
            % ((BELOW_RANGE_MIN, RANGE_MIN, ABOVE_RANGE_MAX, RANGE_MAX)
               + outside + missing
               + (position_difference, math.degrees(heading_difference))
               + self.expected
               + (position_bound, math.degrees(heading_bound))))

        self.assertLessEqual(position_difference, position_bound, message)
        self.assertLessEqual(heading_difference, heading_bound, message)


@launch_testing.post_shutdown_test()
class TestRangeLimitsShutdown(unittest.TestCase):

    def test_the_node_exited_cleanly(self, proc_info, cbgl):
        launch_testing.asserts.assertExitCodes(proc_info, process=cbgl)
