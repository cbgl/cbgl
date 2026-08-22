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
A map read from a file on disk, and a node that subscribes to no map at all.

The other files here publish the fixture room on a topic. This one writes the
same room to disk as a map description and a PGM beside it, points the node at
the description, and requires the same answer out of it. Nothing else about the
launch differs, so what the answer proves is that the file route ends where the
topic route ends: one map, one localiser, one path through the node.

**The room is written at a translated and turned origin.** The description
carries the grid's own origin, which for this fixture is neither at the world
origin nor square to it, so a description read as though a map were flat and
centred fails every case here. The picture is written from the last row of the
grid to the first, because a PGM's first row is the top of the picture and a
grid's first row is the one at the origin, and a map read the wrong way up is a
reflection of a building that does not exist.

**No map topic is subscribed, and that is asserted of the node.** A node that
read the file and subscribed anyway would pass every other case in this file
while still waiting to be handed a second map, so the subscriptions the node
actually carries are read from the graph and required to carry no occupancy grid
among them.

**The files live in a directory of this launch's own** and are taken away once
the launch is over. A map description is two files that have to agree, and a
fixture pair committed beside the room's own data would be a third copy of the
same room, free to drift from the other two.
"""

import shutil
import tempfile
import unittest

from cbgl_test_support import (
    GLOBAL_FRAME,
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
    write_map_file,
    yaw_of,
)
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.node import NodeNameNonExistentError

CELLS = 4.0

GRID = 'pillar_room_grid_rotated.csv'

NODE_NAME = 'cbgl_from_file'
POSE_TOPIC = '/from_file/pose'

OCCUPANCY_GRID_TYPE = 'nav_msgs/msg/OccupancyGrid'

DIRECTORY = []


@pytest.mark.launch_test
def generate_test_description():
    """Launch one node whose map is a file rather than a topic."""
    DIRECTORY.append(tempfile.mkdtemp(prefix='cbgl_map_from_file_'))

    description = write_map_file(DIRECTORY[0], read_grid(GRID))

    node = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name=NODE_NAME,
        output='screen',
        parameters=[node_parameters(map_topic='', map_file=description,
                                    output_pose_topic=POSE_TOPIC)],
    )
    return (
        launch.LaunchDescription(
            [node, keep_alive(), launch_testing.actions.ReadyToTest()]),
        {'cbgl_from_file': node},
    )


class TestMapFromFile(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.grid = read_grid(GRID)
        cls.scan = read_scans()[0]
        cls.truth = world_from_room(cls.grid, read_truth()[0])
        cls.expected = robot_from_sensor(cls.truth)

        cls.harness = Harness(node_name=NODE_NAME,
                              output_pose_topic=POSE_TOPIC,
                              map_topic='')
        cls.harness.wait_for_connection()
        cls.harness.publish_sensor_placement()
        cls.harness.spin(1.0)
        cls.harness.publish_scan(cls.scan)
        cls.harness.spin(0.5)

        cls.response = cls.harness.call()
        cls.harness.spin(1.0)

        cls.poses = list(cls.harness.poses)

    @classmethod
    def tearDownClass(cls):
        cls.harness.destroy_node()
        rclpy.shutdown()

    def answer(self):
        """Return the one pose the localisation of setUpClass published."""
        self.assertTrue(self.response.success, self.response.message)
        self.assertEqual(len(self.poses), 1,
                         'the call published %d poses where one was asked for'
                         % len(self.poses))
        return self.poses[0]

    def test_1_the_map_in_the_file_is_the_map_the_answer_is_found_in(self):
        pose = self.answer().pose.pose
        tolerance = CELLS * self.grid.resolution

        self.assertAlmostEqual(
            pose.position.x, self.expected[0], delta=tolerance,
            msg='the scan was cast from a sensor at (%.4f, %.4f, %.4f) of the '
                'room the file describes, which puts the robot at (%.4f, %.4f, '
                '%.4f) through the placement in the transform tree. The node '
                'placed it at (%.4f, %.4f, %.4f). The bound is %g cells of %g '
                'metres, and a miss of the whole map is a picture read upside '
                'down or an origin left at nought'
                % (self.truth + self.expected
                   + (pose.position.x, pose.position.y,
                      yaw_of(pose.orientation))
                   + (CELLS, self.grid.resolution)))
        self.assertAlmostEqual(pose.position.y, self.expected[1],
                               delta=tolerance)

        self.assertAlmostEqual(
            wrap(yaw_of(pose.orientation) - self.expected[2]), 0.0,
            delta=heading_tolerance(self.scan, self.grid.resolution, CELLS),
            msg='the robot faces %.4f radians and the node reported %.4f. The '
                'bound is %g cells seen at the mean range this scan carries'
                % (self.expected[2], yaw_of(pose.orientation), CELLS))

    def test_2_the_answer_is_labelled_with_the_configured_global_frame(self):
        """
        Label the answer with the frame the settings name.

        A description file states no frame, so the node is the only thing that
        knows which one the map it just read is expressed in. An answer labelled
        anything else is an answer whose coordinates nobody can place.
        """
        frame_id = self.answer().header.frame_id

        self.assertEqual(
            frame_id, GLOBAL_FRAME,
            'the answer is labelled %r where the configured global frame is '
            '%r' % (frame_id, GLOBAL_FRAME))

    def test_3_nothing_is_subscribed_that_carries_a_map(self):
        try:
            subscriptions = self.harness.get_subscriber_names_and_types_by_node(
                NODE_NAME, '/')
        except NodeNameNonExistentError:
            self.fail('%s is not in the graph, so the node either never came '
                      'up or came up under another name' % NODE_NAME)

        carrying_a_map = [name for name, types in subscriptions
                          if OCCUPANCY_GRID_TYPE in types]

        self.assertEqual(
            carrying_a_map, [],
            'the node read its map from a file and subscribed to %r as well. '
            'Whatever arrives there would replace the map that was read, which '
            'is not what an operator who named a file asked for'
            % carrying_a_map)


@launch_testing.post_shutdown_test()
class TestMapFromFileShutdown(unittest.TestCase):

    def test_the_node_exited_cleanly(self, proc_info, cbgl_from_file):
        shutil.rmtree(DIRECTORY[0], ignore_errors=True)

        launch_testing.asserts.assertExitCodes(
            proc_info, process=cbgl_from_file)
