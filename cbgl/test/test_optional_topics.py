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
of anything this file decides. The cast is required to be as long as the scan
the sensor reported and to agree with the measurement to within a few cells,
because a cast from where the answer says the sensor stood is what it claims to
be.

**The cast is laid over three more scans.** Two more nodes are launched, each on
topics of its own so that what they are handed reaches nobody else. One takes
every ray it is handed and runs twice: on a panoramic scan from a sensor mounted
upside down, and on a scan of three quarters of a turn symmetric about the
sensor's heading, the way a real sensor reports one. The other reduces a
panoramic scan onto half its rays before it searches. The cast is drawn in the
frame the scan names, at the angles the scan carries, over the scan the sensor
reported, so in all three it is held to that frame, that first angle, that
spacing and that ray count, and to agreeing with the scan ray for ray.

**Absence is asserted of the node rather than of a topic.** A publication that
was never asked for has no topic name to listen to, so what is checked is that
the node carries no publisher of either type at all.
"""

import math
import unittest

from cbgl_test_support import (
    build_scan,
    distance_to_free_space,
    FIRST_RAY_ANGLE,
    free_space_of,
    GLOBAL_FRAME,
    Harness,
    hypothesis_count,
    keep_alive,
    node_parameters,
    ORIENTATIONS,
    read_grid,
    read_scans,
    ROBOT_FRAME,
    SCAN_SIZE,
    SCAN_STAMP,
    SENSOR_ON_ROBOT,
    TOP_K_CAERS,
    upside_down,
)
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.node import NodeNameNonExistentError
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan

CELLS = 4.0
OVERLAY_CELLS = 2.0

HYPOTHESES_TOPIC = '/named/hypotheses'
MAP_SCAN_TOPIC = '/named/map_scan'
NAMED_POSE_TOPIC = '/named/pose'
QUIET_POSE_TOPIC = '/quiet/pose'
TOP_HYPOTHESES_TOPIC = '/named/top_hypotheses'

OVERLAID_MAP_SCAN_TOPIC = '/overlaid/map_scan'
OVERLAID_MAP_TOPIC = '/overlaid/map'
OVERLAID_POSE_TOPIC = '/overlaid/pose'
OVERLAID_SCAN_TOPIC = '/overlaid/scan'

REDUCED_MAP_SCAN_TOPIC = '/reduced/map_scan'
REDUCED_MAP_TOPIC = '/reduced/map'
REDUCED_POSE_TOPIC = '/reduced/pose'
REDUCED_SCAN_TOPIC = '/reduced/scan'

NARROW_RAYS = 271
REDUCED_RAYS = SCAN_SIZE // 2

UPSIDE_DOWN = math.pi
UPSIDE_DOWN_SENSOR_FRAME = 'upside_down_sensor_frame'

POSE_ARRAY_TYPE = 'geometry_msgs/msg/PoseArray'
SCAN_TYPE = 'sensor_msgs/msg/LaserScan'


@pytest.mark.launch_test
def generate_test_description():
    """Launch the nodes the optional topics are asked of, named and not."""
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
    overlaid = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name='cbgl_overlaid',
        output='screen',
        parameters=[node_parameters(
            size_scan=0,
            map_topic=OVERLAID_MAP_TOPIC,
            scan_topic=OVERLAID_SCAN_TOPIC,
            output_pose_topic=OVERLAID_POSE_TOPIC,
            map_scan_topic=OVERLAID_MAP_SCAN_TOPIC)],
    )
    reduced = launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name='cbgl_reduced',
        output='screen',
        parameters=[node_parameters(
            size_scan=REDUCED_RAYS,
            map_topic=REDUCED_MAP_TOPIC,
            scan_topic=REDUCED_SCAN_TOPIC,
            output_pose_topic=REDUCED_POSE_TOPIC,
            map_scan_topic=REDUCED_MAP_SCAN_TOPIC)],
    )
    return (
        launch.LaunchDescription(
            [quiet, named, overlaid, reduced, keep_alive(),
             launch_testing.actions.ReadyToTest()]),
        {'cbgl': quiet, 'cbgl_named': named, 'cbgl_overlaid': overlaid,
         'cbgl_reduced': reduced},
    )


def ray_differences(cast, measured):
    """Return how far apart two scans are, ray for ray, where both measured."""
    return [abs(cast_range - measured_range)
            for cast_range, measured_range in zip(cast, measured)
            if cast_range > 0.0 and measured_range > 0.0]


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

        cls.overlaid = Harness(node_name='cbgl_overlaid',
                               output_pose_topic=OVERLAID_POSE_TOPIC,
                               map_scan_topic=OVERLAID_MAP_SCAN_TOPIC,
                               map_topic=OVERLAID_MAP_TOPIC,
                               scan_topic=OVERLAID_SCAN_TOPIC)
        cls.reduced = Harness(node_name='cbgl_reduced',
                              output_pose_topic=REDUCED_POSE_TOPIC,
                              map_scan_topic=REDUCED_MAP_SCAN_TOPIC,
                              map_topic=REDUCED_MAP_TOPIC,
                              scan_topic=REDUCED_SCAN_TOPIC)

        for harness in (cls.overlaid, cls.reduced):
            harness.wait_for_connection()
            harness.publish_map(cls.grid)

        cls.overlaid.publish_transform(ROBOT_FRAME, UPSIDE_DOWN_SENSOR_FRAME,
                                       SENSOR_ON_ROBOT, roll=UPSIDE_DOWN)
        cls.overlaid.spin(1.0)

        increment = 2.0 * math.pi / len(cls.scan)
        first_ray_angle = -0.5 * (NARROW_RAYS - 1) * increment
        prefix = round((first_ray_angle - FIRST_RAY_ANGLE) / increment)

        cls.overlays = {
            'upside down': cls.overlay(
                cls.overlaid, upside_down(cls.scan),
                frame_id=UPSIDE_DOWN_SENSOR_FRAME),
            'narrower': cls.overlay(
                cls.overlaid, cls.scan[prefix:prefix + NARROW_RAYS],
                first_ray_angle=first_ray_angle, increment=increment),
            'reduced': cls.overlay(cls.reduced, cls.scan),
        }

    @classmethod
    def tearDownClass(cls):
        cls.reduced.destroy_node()
        cls.overlaid.destroy_node()
        cls.named.destroy_node()
        cls.quiet.destroy_node()
        rclpy.shutdown()

    @classmethod
    def overlay(cls, harness, ranges, **shape):
        """
        Localise one scan and return the answer, the scan and its casts.

        The scan is returned as the node receives it, through serialisation
        and back, because its angles travel as 32 bit floats and the cast is
        held to carry exactly those.
        """
        harness.clear()
        harness.publish_scan(ranges, **shape)
        harness.spin(0.5)

        response = harness.call()
        harness.spin(1.0)

        received = deserialize_message(
            serialize_message(build_scan(SCAN_STAMP, ranges, **shape)),
            LaserScan)

        return response, received, list(harness.map_scans)

    def assert_laid_over(self, case):
        """
        Require the cast of one run to lie over its scan ray for ray.

        The bound is two cells a ray on average, tighter than the panoramic
        case's four cells twice over, because that bound separates nothing
        here. Measured on this fixture, every run alike: a cast that lies over
        its scan disagrees with it by 0.0239 m upside down, 0.0249 m on the
        narrower scan and 0.0239 m on the reduced one. Before the cast was
        repaired the upside down one disagreed by 0.3108 m, being the mirror
        image of its scan, and the narrower one by 0.1907 m, being turned and
        stretched, both inside 0.4 m. Two cells is 0.1 m.
        """
        response, measured, casts = self.overlays[case]

        self.assertTrue(response.success, response.message)
        self.assertEqual(len(casts), 1,
                         'the run on a scan %s published %d casts where one '
                         'was asked for' % (case, len(casts)))

        published = casts[0]
        differences = ray_differences(published.ranges, measured.ranges)
        disagreement = (sum(differences) / len(differences)
                        if differences else math.inf)
        bound = OVERLAY_CELLS * self.grid.resolution

        message = (
            'on a scan %s the cast carries %d rays from %.4f radians, %.6f '
            'apart, in "%s", and the scan it is laid over carries %d from '
            '%.4f, %.6f apart, in "%s". Ray for ray the two disagree by %.4f '
            'metres on average, against a bound of %.4f'
            % (case, len(published.ranges), published.angle_min,
               published.angle_increment, published.header.frame_id,
               len(measured.ranges), measured.angle_min,
               measured.angle_increment, measured.header.frame_id,
               disagreement, bound))

        self.assertEqual(published.header.frame_id, measured.header.frame_id,
                         message)
        self.assertEqual(len(published.ranges), len(measured.ranges), message)
        self.assertEqual(published.angle_min, measured.angle_min, message)
        self.assertEqual(published.angle_increment, measured.angle_increment,
                         message)
        self.assertLess(disagreement, bound, message)

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
            'the sensor reported %d rays and the cast carries %d, so the two '
            'cannot be laid over each other'
            % (SCAN_SIZE, len(published.ranges)))

        self.assertEqual(
            (published.header.stamp.sec, published.header.stamp.nanosec),
            SCAN_STAMP)

        compared = ray_differences(published.ranges, self.scan)

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

    def test_5_the_map_scan_lies_over_a_sensor_mounted_upside_down(self):
        """
        Order the cast the way the rolled sensor orders its own rays.

        The scan is handed to the search as an upright sensor's, its ranges
        reversed, so the pose the search answers with is that upright sensor's.
        A cast taken from it counterclockwise and published in the rolled frame
        is drawn as the mirror image of what the sensor saw.

        **What this does not catch, measured.** A cast taken clockwise from the
        answer's heading, without the turn the upright sensor's heading carries,
        is one ray out of turn on this scan. It disagrees with the scan by
        0.0287 m a ray on average, where the right cast disagrees by 0.0239 m.
        Both are well inside the bound, so this case cannot tell them apart.
        """
        self.assert_laid_over('upside down')

    def test_6_the_map_scan_lies_over_a_sensor_that_sees_less_than_a_turn(self):
        """
        Start the cast at the scan's own first ray and space it as the scan is.

        The 271 rays run from -135 to 135 degrees, symmetric about the heading,
        the way a real sensor reports a scan. A cast that begins half a turn
        behind the heading and spreads its rays over a whole turn, published
        under the scan's own first angle, is drawn turned and stretched.
        """
        self.assert_laid_over('narrower')

    def test_7_the_map_scan_lies_over_a_scan_searched_on_fewer_rays(self):
        """
        Cast at the rays the sensor reported, not at the rays searched on.

        The search runs on 180 of the 360 rays. Nobody sees that reduced scan,
        and the cast is laid over the one the sensor reported, so it carries
        that scan's ray count and spacing whatever size_scan says.
        """
        self.assert_laid_over('reduced')


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_the_nodes_exited_cleanly(self, proc_info, cbgl, cbgl_named,
                                      cbgl_overlaid, cbgl_reduced):
        for process in (cbgl, cbgl_named, cbgl_overlaid, cbgl_reduced):
            launch_testing.asserts.assertExitCodes(proc_info, process=process)
