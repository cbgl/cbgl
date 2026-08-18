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
The three publications that exist to be looked at, absent until they are named.

Every hypothesis drawn, the survivors of the sift and the scan the answer would
see are of no use to a robot and of great use to somebody working out why an
answer was what it was. They cost bandwidth and they cost the drawing of a
message, so each is off until its topic is given a name.

Two nodes are launched from one map and one scan: one left as it comes, which
must publish nothing but its answer, and one with all three names given, which
must publish all three. Both run once.

**What the three are held to.** The hypothesis count is computed here from the
grid's own free area and the density the node is launched with, since the
dispersal draws round(density * free area) of them. Every hypothesis and every
survivor is required to stand at the centre of a free cell of that same grid,
which is what the dispersal draws and is a property of the fixture rather than
of anything this file decides. The cast is required to be as long as the scan the
run was reduced onto and to agree with the measurement to within a few cells,
because a cast from where the answer says the sensor stood is what it claims to
be.

**Absence is asserted of the node rather than of a topic.** A publication that
was never asked for has no topic name to listen to, so what is checked is that
the node carries no publisher of either type at all.
"""

import unittest

from cbgl_test_support import (
    distance_to_free_space,
    free_space_of,
    GLOBAL_FRAME,
    Harness,
    hypothesis_count,
    keep_alive,
    node_parameters,
    ORIENTATIONS,
    read_grid,
    read_scans,
    SCAN_SIZE,
    SCAN_STAMP,
    TOP_K_CAERS,
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

HYPOTHESES_TOPIC = '/named/hypotheses'
MAP_SCAN_TOPIC = '/named/map_scan'
NAMED_POSE_TOPIC = '/named/pose'
QUIET_POSE_TOPIC = '/quiet/pose'
TOP_HYPOTHESES_TOPIC = '/named/top_hypotheses'

POSE_ARRAY_TYPE = 'geometry_msgs/msg/PoseArray'
SCAN_TYPE = 'sensor_msgs/msg/LaserScan'


@pytest.mark.launch_test
def generate_test_description():
    """Launch one node with the optional topics named and one without."""
    quiet = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name='cbgl',
        output='screen',
        parameters=[node_parameters(output_pose_topic=QUIET_POSE_TOPIC)],
    )
    named = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name='cbgl_named',
        output='screen',
        parameters=[node_parameters(
            output_pose_topic=NAMED_POSE_TOPIC,
            hypotheses_topic=HYPOTHESES_TOPIC,
            top_hypotheses_topic=TOP_HYPOTHESES_TOPIC,
            map_scan_topic=MAP_SCAN_TOPIC)],
    )
    return (
        launch.LaunchDescription(
            [quiet, named, keep_alive(),
             launch_testing.actions.ReadyToTest()]),
        {'cbgl': quiet, 'cbgl_named': named},
    )


class TestOptionalTopics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.grid = read_grid()
        cls.scan = read_scans()[0]

        cls.quiet = Harness(node_name='cbgl',
                            output_pose_topic=QUIET_POSE_TOPIC)
        cls.named = Harness(node_name='cbgl_named',
                            output_pose_topic=NAMED_POSE_TOPIC,
                            hypotheses_topic=HYPOTHESES_TOPIC,
                            top_hypotheses_topic=TOP_HYPOTHESES_TOPIC,
                            map_scan_topic=MAP_SCAN_TOPIC)

        cls.quiet.wait_for_connection(subscribers=2)
        cls.quiet.publish_sensor_placement()
        cls.quiet.publish_map(cls.grid)
        cls.quiet.spin(1.0)
        cls.quiet.publish_scan(cls.scan)
        cls.quiet.spin(0.5)

        cls.quiet_outcome = cls.quiet.call()
        cls.quiet.spin(1.0)

        cls.named.spin(0.5)
        cls.named_outcome = cls.named.call()
        cls.named.spin(1.0)

    @classmethod
    def tearDownClass(cls):
        cls.named.destroy_node()
        cls.quiet.destroy_node()
        rclpy.shutdown()

    def assert_stands_on_free_space(self, poses, label):
        """Require every pose of an array to sit at the centre of a free cell."""
        for index, pose in enumerate(poses):
            distance = distance_to_free_space(
                self.grid, pose.position.x, pose.position.y)

            self.assertLess(
                distance, 1e-6,
                '%s %d stands at (%.4f, %.4f), which is %.4f metres from the '
                'nearest free cell centre of the map it was drawn over. A '
                'hypothesis is the centre of a free cell and nothing else'
                % (label, index, pose.position.x, pose.position.y, distance))

    def publications_of(self, node_name, timeout=30.0):
        """Return what a node publishes, by topic and by type."""
        for _ in range(int(timeout / 0.2)):
            try:
                found = self.quiet.get_publisher_names_and_types_by_node(
                    node_name, '/')
            except NodeNameNonExistentError:
                found = []

            if found:
                return found

            self.quiet.spin(0.2)

        self.fail('%s publishes nothing at all, so it is not on the graph'
                  % node_name)

    def test_1_nothing_optional_is_published_when_nothing_is_named(self):
        self.assertTrue(self.quiet_outcome.success, self.quiet_outcome.message)
        self.assertEqual(len(self.quiet.poses), 1,
                         'the node that publishes nothing optional published '
                         'no answer either, so this case would hold of a node '
                         'that never ran')

        for topic, types in self.publications_of('cbgl'):
            self.assertNotIn(POSE_ARRAY_TYPE, types,
                             'the node publishes poses on %s though neither '
                             'hypotheses topic was named' % topic)
            self.assertNotIn(SCAN_TYPE, types,
                             'the node publishes a scan on %s though no map '
                             'scan topic was named' % topic)

    def test_2_every_hypothesis_is_published_when_the_topic_is_named(self):
        self.assertTrue(self.named_outcome.success, self.named_outcome.message)
        self.assertEqual(len(self.named.hypotheses), 1,
                         'the run published %d hypothesis sets on %s where one '
                         'was asked for'
                         % (len(self.named.hypotheses), HYPOTHESES_TOPIC))

        published = self.named.hypotheses[0]
        expected = hypothesis_count(self.grid)

        self.assertEqual(
            len(published.poses), expected,
            'the map offers %.4f square metres of free space, which at the '
            'density this node was launched with is %d hypotheses, and %d were '
            'published'
            % (free_space_of(self.grid), expected, len(published.poses)))

        self.assertEqual(published.header.frame_id, GLOBAL_FRAME)
        self.assertEqual(
            (published.header.stamp.sec, published.header.stamp.nanosec),
            SCAN_STAMP)

        self.assert_stands_on_free_space(published.poses, 'hypothesis')

    def test_3_the_survivors_are_published_when_the_topic_is_named(self):
        self.assertEqual(len(self.named.top_hypotheses), 1,
                         'the run published %d survivor sets on %s where one '
                         'was asked for'
                         % (len(self.named.top_hypotheses),
                            TOP_HYPOTHESES_TOPIC))

        published = self.named.top_hypotheses[0]

        self.assertEqual(
            len(published.poses), TOP_K_CAERS,
            'the sift was asked to keep %d of the %d candidates this map and '
            'this density offer, and %d were published'
            % (TOP_K_CAERS, hypothesis_count(self.grid) * ORIENTATIONS,
               len(published.poses)))

        self.assertEqual(published.header.frame_id, GLOBAL_FRAME)
        self.assertEqual(
            (published.header.stamp.sec, published.header.stamp.nanosec),
            SCAN_STAMP)

        self.assert_stands_on_free_space(published.poses, 'survivor')

    def test_4_the_map_scan_is_published_when_the_topic_is_named(self):
        self.assertEqual(len(self.named.map_scans), 1,
                         'the run published %d cast scans on %s where one was '
                         'asked for'
                         % (len(self.named.map_scans), MAP_SCAN_TOPIC))

        published = self.named.map_scans[0]

        self.assertEqual(
            len(published.ranges), SCAN_SIZE,
            'the run was reduced onto %d rays and the cast carries %d, so the '
            'two cannot be laid over each other'
            % (SCAN_SIZE, len(published.ranges)))

        self.assertEqual(
            (published.header.stamp.sec, published.header.stamp.nanosec),
            SCAN_STAMP)

        compared = [abs(cast - measured)
                    for cast, measured in zip(published.ranges, self.scan)
                    if cast > 0.0 and measured > 0.0]

        self.assertTrue(compared, 'the cast shares no usable ray with the '
                                  'measurement it is meant to be laid over')

        bound = 2.0 * CELLS * self.grid.resolution
        disagreement = sum(compared) / len(compared)

        self.assertLess(
            disagreement, bound,
            'the cast disagrees with the measurement by %.4f metres a ray on '
            'average, against a bound of %.4f. The answer stands within %g '
            'cells of the truth and a cast into a raster disagrees with one '
            'from the polygons of the same room by about a cell, so a cast '
            'from where the answer says the sensor stood cannot be further '
            'out than that twice over' % (disagreement, bound, CELLS))


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_the_nodes_exited_cleanly(self, proc_info, cbgl, cbgl_named):
        for process in (cbgl, cbgl_named):
            launch_testing.asserts.assertExitCodes(proc_info, process=process)
