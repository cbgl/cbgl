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
A sensor mounted upside down, which only the transform tree says it is.

A lidar hung beneath a plate is rolled half a turn about its forward axis.
Forward stays forward and left becomes right, so the ray such a sensor calls a
points where an upright one would call -a, and the scan it reports is the mirror
image of the scan an upright sensor in its place would report. Nothing in the
scan says so and no setting of this node does. The rotation of the sensor's
frame in the transform tree is the only place it is written, so a node that
keeps the heading of that rotation and drops the rest takes the sensor for an
upright one and localises the mirror image of what was seen.

**One physical scan, two mountings.** The fixture scan is published twice from
the same place: once as an upright sensor reports it, in a frame mounted
upright, and once as the same sensor reports it upside down, in a frame mounted
with the same translation and heading and half a turn of roll. The robot stands
in the same place both times, so both answers are held to the same pose and to
each other. The upright case is the control. Without it a failure of the other
could be the fixture's and not the mounting's.

**The placement is the one every node level file uses.** It is turned as well
as shifted, so a node that lost the heading for the rolled frame alone would
miss by more than the bar. Its reach is less than half a metre, so a node that
dropped the translation for the rolled frame alone would still pass the bar,
and it is the agreement with the control that catches it.

**Why a mirrored scan cannot pass here by luck.** The room is a rectangle with
one pillar. A rectangle is its own mirror image about either of its axes, but
the pillar stands on neither, so the mirror image of the room is a different
room. The fixture's sensor stands on neither axis and faces along neither, so
the mirrored scan is not the scan itself either.

**What the answers are held to.** Half a metre and five degrees, the bar this
package judges a localisation by. The four cell bound of the other node level
files is tighter in position, but on this scan it allows more than five degrees
of heading, so it is not used here. The upside down answer is also held to the
upright one, far more tightly, because a repair that is nearly right passes the
bar. The case that compares them says how tightly and why.
"""

import math
import unittest

from cbgl_test_support import (
    FIRST_RAY_ANGLE,
    Harness,
    keep_alive,
    node_parameters,
    read_grid,
    read_scans,
    read_truth,
    ROBOT_FRAME,
    robot_from_sensor,
    SENSOR_FRAME,
    SENSOR_ON_ROBOT,
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

HEADING_BAR = math.radians(5.0)
POSITION_BAR = 0.5

UPSIDE_DOWN = math.pi
UPSIDE_DOWN_SENSOR_FRAME = 'upside_down_sensor_frame'


@pytest.mark.launch_test
def generate_test_description():
    """Launch one node, to be handed a scan from each of two mountings."""
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


def upside_down(ranges):
    """
    Return a panoramic scan as the same sensor reports it upside down.

    The rolled sensor's ray at angle a reads what the upright sensor's ray at
    -a reads, so each ray is looked up by its angle. The list is not simply
    reversed: a panoramic scan whose first ray is at minus pi is not symmetric
    about nought, and reversing it would turn the scan by a ray as well as
    mirror it.
    """
    size = len(ranges)
    increment = 2.0 * math.pi / size

    return [ranges[round((-2.0 * FIRST_RAY_ANGLE - ray * increment)
                         / increment) % size]
            for ray in range(size)]


class TestUpsideDownSensor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.grid = read_grid()
        cls.scan = read_scans()[0]
        cls.truth = world_from_room(cls.grid, read_truth()[0])
        cls.expected = robot_from_sensor(cls.truth)

        cls.harness = Harness()
        cls.harness.wait_for_connection()
        cls.harness.publish_sensor_placement()
        cls.harness.publish_transform(ROBOT_FRAME, UPSIDE_DOWN_SENSOR_FRAME,
                                      SENSOR_ON_ROBOT, roll=UPSIDE_DOWN)
        cls.harness.publish_map(cls.grid)
        cls.harness.spin(1.0)

        cls.outcomes = {
            'upright': cls.localise(cls.scan, SENSOR_FRAME),
            'upside down': cls.localise(upside_down(cls.scan),
                                        UPSIDE_DOWN_SENSOR_FRAME),
        }

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    @classmethod
    def localise(cls, ranges, frame_id):
        """Localise one scan and return the response and the poses published."""
        cls.harness.clear()
        cls.harness.publish_scan(ranges, frame_id=frame_id)
        cls.harness.spin(0.5)

        response = cls.harness.call()
        cls.harness.spin(1.0)

        return response, list(cls.harness.poses)

    def answer(self, mounting):
        """Return the one robot pose published for a mounting, as x, y, yaw."""
        response, poses = self.outcomes[mounting]

        self.assertTrue(response.success, response.message)
        self.assertEqual(len(poses), 1,
                         'the call published %d poses where one was asked for'
                         % len(poses))

        pose = poses[0].pose.pose
        return (pose.position.x, pose.position.y, yaw_of(pose.orientation))

    def assert_localised(self, mounting):
        """Require the robot where the scan was cast from."""
        found = self.answer(mounting)

        position_error = math.hypot(found[0] - self.expected[0],
                                    found[1] - self.expected[1])
        heading_error = abs(wrap(found[2] - self.expected[2]))

        message = (
            'a sensor mounted %s put the robot at (%.4f, %.4f, %.4f), %.4f m '
            'and %.2f degrees from where it stands at (%.4f, %.4f, %.4f). The '
            'bar is %g m and %g degrees'
            % ((mounting,) + found
               + (position_error, math.degrees(heading_error))
               + self.expected
               + (POSITION_BAR, math.degrees(HEADING_BAR))))

        self.assertLessEqual(position_error, POSITION_BAR, message)
        self.assertLessEqual(heading_error, HEADING_BAR, message)

    def test_1_a_sensor_mounted_upright_is_localised(self):
        self.assert_localised('upright')

    def test_2_a_sensor_mounted_upside_down_is_localised(self):
        """
        Read the roll of the sensor's frame, not only its heading.

        A node that keeps the heading alone reads this scan as an upright
        sensor's, which is the mirror image of the room the scan was taken in.
        """
        self.assert_localised('upside down')

    def test_3_both_mountings_put_the_robot_in_the_same_place(self):
        """
        Hold the upside down answer to the upright one, not only to the bar.

        The repair reverses the ranges and turns the sensor's heading by the
        sum of the scan's first and last angles, which on a panoramic scan
        beginning at minus pi is minus one ray. A reversal that leaves out the
        turn hands the localiser the same ranges, so it finds the same sensor
        pose, and then places the robot about it one ray of heading away. That
        answer is one ray off in heading, and off in position by the
        placement's reach swung through one ray, and both are well inside the
        bar. The bound is half of each.

        Measured at this placement, three runs of each and every run alike:
        the repair as committed answers 0.0000001 m and 0.000013 degrees from
        the control, and a plain reversal of the ranges answers 0.0075 m and
        1.00 degree from it. The bound comes to 0.0038 m and 0.5 degrees.
        """
        upright = self.answer('upright')
        inverted = self.answer('upside down')

        heading_bound = math.pi / len(self.scan)
        position_bound = (math.hypot(SENSOR_ON_ROBOT[0], SENSOR_ON_ROBOT[1])
                          * heading_bound)

        position_difference = math.hypot(inverted[0] - upright[0],
                                         inverted[1] - upright[1])
        heading_difference = abs(wrap(inverted[2] - upright[2]))

        message = (
            'the sensor mounted upside down put the robot at (%.4f, %.4f, '
            '%.4f) and the one mounted upright at (%.4f, %.4f, %.4f), %.4f m '
            'and %.3f degrees apart. The bound is %.4f m and %.3f degrees, '
            'half of what turning the robot about the sensor by one ray moves '
            'it'
            % (inverted + upright
               + (position_difference, math.degrees(heading_difference),
                  position_bound, math.degrees(heading_bound))))

        self.assertLessEqual(position_difference, position_bound, message)
        self.assertLessEqual(heading_difference, heading_bound, message)


@launch_testing.post_shutdown_test()
class TestUpsideDownSensorShutdown(unittest.TestCase):

    def test_the_node_exited_cleanly(self, proc_info, cbgl):
        launch_testing.asserts.assertExitCodes(proc_info, process=cbgl)
