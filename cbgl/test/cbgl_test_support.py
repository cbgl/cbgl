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
Shared machinery for the node level tests.

The room is the one the library's own tests are held to, read from the same
three files under test/fixtures: a grid of one room at five centimetre cells, a
scan cast from that room's own polygons at each of three known poses, and those
poses. Nothing here draws a room of its own, so what the node is asked to
localise in is data both levels share rather than two rooms that have to be
kept in step.

Every number a test compares against is computed here from the grid, the scan
or the placement, and never written down: the hypothesis count from the grid's
free area, the tolerances from the grid's cell size, and the robot pose from the
sensor pose the fixture names and the placement the fixture puts in the
transform tree.

The placement is deliberately larger than the tolerance a recovered pose is
held to, so a node that published the sensor's pose where the robot's was asked
for, or that dropped the placement altogether, fails rather than passes by
coincidence.
"""

import collections
import math
import os
import time

from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, TransformStamped
import launch
import launch.actions
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fixtures')

FINE_GRID = 'pillar_room_grid_fine.csv'
SCANS = 'pillar_room_scans.csv'
TRUTH = 'pillar_room_truth.csv'

Grid = collections.namedtuple(
    'Grid', ['width', 'height', 'resolution', 'origin', 'cells'])

DENSITY = 15.0
ORIENTATIONS = 32
TOP_K_CAERS = 10

OCCUPANCY_THRESHOLD = 65

FIRST_RAY_ANGLE = -math.pi
RANGE_MAX = 100.0
SCAN_SIZE = 360

GLOBAL_FRAME = 'world_frame'
ODOM_FRAME = 'odometry_frame'
ROBOT_FRAME = 'base_footprint'
SENSOR_FRAME = 'sensor_frame'

SENSOR_ON_ROBOT = (0.35, -0.25, 0.30)
ROBOT_IN_ODOMETRY = (1.70, -0.60, -0.40)

MAP_STAMP = (1700000000, 150000000)
SCAN_STAMP = (1700000000, 250000000)

MAP_TOPIC = '/map'
SCAN_TOPIC = '/scan'

SERVICE_TIMEOUT = 240.0


def build_map(stamp, grid, frame_id=GLOBAL_FRAME):
    """Return one of the fixture grids as an occupancy grid message."""
    message = OccupancyGrid()
    message.header.stamp.sec, message.header.stamp.nanosec = stamp
    message.header.frame_id = frame_id
    message.info.map_load_time.sec, message.info.map_load_time.nanosec = stamp
    message.info.resolution = grid.resolution
    message.info.width = grid.width
    message.info.height = grid.height
    message.info.origin.position.x = grid.origin[0]
    message.info.origin.position.y = grid.origin[1]
    message.info.origin.orientation.z = math.sin(grid.origin[2] / 2.0)
    message.info.origin.orientation.w = math.cos(grid.origin[2] / 2.0)
    message.data = list(grid.cells)
    return message


def build_scan(stamp, ranges, frame_id=SENSOR_FRAME,
               first_ray_angle=FIRST_RAY_ANGLE):
    """
    Return a panoramic scan of the given ranges, stamped as asked.

    The rays span exactly one turn and ray zero sits at first_ray_angle, which
    defaults to minus pi because that is where the matcher's own ray contract
    puts it. A scan spanning one turn from anywhere else is a legal scan and a
    different scan, and giving that angle is how a test says so on purpose.
    """
    size = len(ranges)
    increment = 2.0 * math.pi / size

    scan = LaserScan()
    scan.header.stamp.sec, scan.header.stamp.nanosec = stamp
    scan.header.frame_id = frame_id
    scan.angle_min = first_ray_angle
    scan.angle_max = first_ray_angle + 2.0 * math.pi - increment
    scan.angle_increment = increment
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = 0.0
    scan.range_max = RANGE_MAX
    scan.ranges = [float(r) for r in ranges]
    scan.intensities = []
    return scan


def compose(a, b):
    """Return b expressed in the frame a is expressed in."""
    return (a[0] + math.cos(a[2]) * b[0] - math.sin(a[2]) * b[1],
            a[1] + math.sin(a[2]) * b[0] + math.cos(a[2]) * b[1],
            wrap(a[2] + b[2]))


def distance_to_free_space(grid, x, y):
    """
    Return how far a position stands from the nearest free cell's centre.

    The dispersal draws the centre of a free cell and nothing else, so this is
    zero for a hypothesis and the better part of a cell for anything else.
    """
    return min(math.hypot(x - centre[0], y - centre[1])
               for centre in free_cell_centres(grid))


def free_cell_centres(grid, threshold=OCCUPANCY_THRESHOLD):
    """
    Return the world position of the centre of every free cell of a grid.

    A cell is free when its value is known and below the occupancy threshold,
    which is the rule the map itself applies, and the threshold passed in is the
    one the node under test is configured with.
    """
    centres = []

    for row in range(grid.height):
        for column in range(grid.width):
            value = grid.cells[row * grid.width + column]

            if 0 <= value < threshold:
                centres.append(world_from_room(
                    grid, ((column + 0.5) * grid.resolution,
                           (row + 0.5) * grid.resolution, 0.0))[:2])

    return centres


def free_space_of(grid, threshold=OCCUPANCY_THRESHOLD):
    """Return the free area of a grid, in square metres."""
    return len(free_cell_centres(grid, threshold)) * grid.resolution ** 2


def heading_tolerance(ranges, resolution, cells):
    """
    Return the heading a scan of these ranges can be resolved to.

    A heading wrong by dt moves what the sensor would see at range r sideways by
    r * dt, so a heading is resolved no better than the same bound of cells seen
    at the ranges the scan actually carries.
    """
    usable = [r for r in ranges if 0.0 < r < RANGE_MAX]
    return math.atan(cells * resolution / (sum(usable) / len(usable)))


def hypothesis_count(grid, density=DENSITY):
    """
    Return how many hypotheses a grid of this free area is dispersed over.

    The dispersal draws round(density * free area) positions, so the count is a
    property of the grid and the setting rather than a number to write down.
    """
    return int(math.floor(density * free_space_of(grid) + 0.5))


def invert(pose):
    """Return the pose that undoes pose."""
    return (-(pose[0] * math.cos(pose[2]) + pose[1] * math.sin(pose[2])),
            pose[0] * math.sin(pose[2]) - pose[1] * math.cos(pose[2]),
            wrap(-pose[2]))


def keep_alive():
    """
    Return an action that keeps a launch running while its nodes are dying.

    A launch tears itself down the moment nothing it started is still running,
    and a launch that tore itself down reports one exception in place of every
    assertion the file was written to make. That turns a node which fails to come
    up into a test failure nobody can read. This keeps the launch on its feet
    long enough for the cases to say what they found missing, and is torn down
    with everything else once they have.
    """
    return launch.actions.ExecuteProcess(cmd=['sleep', '3600'], output='log')


def node_parameters(**overrides):
    """
    Return the configuration the node level tests launch the node with.

    Every one of the matcher's settings is the value this package ships, which
    is what the library's own recovery cases are measured at, and the seed is
    pinned so that a launch either recovers every time or never. The global
    frame is deliberately not the default: the answer's frame has to be the one
    configured rather than a name written into the source.
    """
    parameters = {
        'global_frame_id': GLOBAL_FRAME,
        'robot_frame_id': ROBOT_FRAME,
        'odom_frame_id': ODOM_FRAME,
        'dl': DENSITY,
        'da': ORIENTATIONS,
        'top_k_caers': TOP_K_CAERS,
        'rng_seed': 20260818,
        'occupancy_threshold': OCCUPANCY_THRESHOLD,
        'size_scan': SCAN_SIZE,
        'num_iterations': 3,
        'xy_bound': 1.2,
        't_bound': 0.786,
        'max_recoveries': 10,
        'enforce_early_gearup': True,
        'enforce_translation_bound': True,
        'recover_on_failed_sift': True,
    }
    parameters.update(overrides)
    return parameters


def read_grid(name=FINE_GRID):
    """Return one of the room grids under test/fixtures, as its file carries it."""
    with open(os.path.join(FIXTURES, name)) as handle:
        values = handle.read().split()

    width = int(values[0])
    height = int(values[1])

    return Grid(
        width=width,
        height=height,
        resolution=float(values[2]),
        origin=(float(values[3]), float(values[4]), float(values[5])),
        cells=[int(value) for value in values[6:6 + width * height]])


def read_scans(name=SCANS):
    """Return the scans cast from the room's own polygons."""
    with open(os.path.join(FIXTURES, name)) as handle:
        values = handle.read().split()

    count = int(values[0])
    size = int(values[1])

    return [[float(value) for value in values[2 + s * size:2 + (s + 1) * size]]
            for s in range(count)]


def read_truth(name=TRUTH):
    """Return the poses the fixture scans were cast at, in the room's frame."""
    with open(os.path.join(FIXTURES, name)) as handle:
        lines = handle.read().splitlines()

    poses = []

    for line in lines[1:]:
        if line.strip():
            fields = line.split(',')
            poses.append((float(fields[1]), float(fields[2]), float(fields[3])))

    return poses


def robot_from_sensor(sensor, placement=SENSOR_ON_ROBOT):
    """
    Return where the robot is, given where the sensor is and where it sits.

    The fixture's truth is a sensor pose, since the sensor is what casts, and
    the answer the node publishes places the robot. The two differ by the
    placement the transform tree carries, which is the whole of what this undoes.
    """
    return compose(sensor, invert(placement))


def shifted(stamp, seconds):
    """Return a stamp moved by a number of seconds, which may be negative."""
    total = stamp[0] * 10 ** 9 + stamp[1] + int(round(seconds * 10 ** 9))
    return (total // 10 ** 9, total % 10 ** 9)


def transform_message(frame_id, child_frame_id, placement, stamp):
    """Return one transform of the tree, stamped as asked."""
    message = TransformStamped()
    message.header.stamp.sec, message.header.stamp.nanosec = stamp
    message.header.frame_id = frame_id
    message.child_frame_id = child_frame_id
    message.transform.translation.x = placement[0]
    message.transform.translation.y = placement[1]
    message.transform.rotation.z = math.sin(placement[2] / 2.0)
    message.transform.rotation.w = math.cos(placement[2] / 2.0)
    return message


def world_from_room(grid, pose):
    """
    Return a pose of the room placed in the world the grid's origin is given in.

    Every fixture grid draws the same room, each with an origin of its own, so a
    pose of the room becomes a pose of the world through that grid's origin and
    through nothing else.
    """
    return compose(grid.origin, pose)


def wrap(angle):
    """Return angle folded into [-pi, pi)."""
    return math.remainder(angle, 2.0 * math.pi)


def yaw_of(orientation):
    """Return the heading a quaternion carries, about z."""
    return wrap(2.0 * math.atan2(orientation.z, orientation.w))


class Harness(Node):
    """
    Drives the node under test and collects everything it publishes.

    The map and the scan topics can be given per harness, which is what lets one
    launch hold several nodes that are each starved of something different. The
    transform tree cannot be divided that way, being one tree, so a case that
    needs a placement to be missing withholds a placement for the frame its own
    scan names rather than a topic.

    The scan publisher's reliability and queue depth can be given too, and
    default to what a scan is published with everywhere else here. A best effort
    publisher is the only way to ask whether a node configured for best effort
    really subscribed that way: a reliable subscription does not match a best
    effort publisher and receives nothing at all from it, so a setting that never
    reached the subscription shows up as a run with no scan rather than as a
    passing case.
    """

    def __init__(self, node_name='cbgl', output_pose_topic='/initialpose',
                 hypotheses_topic='', top_hypotheses_topic='',
                 map_scan_topic='', map_topic=MAP_TOPIC,
                 scan_topic=SCAN_TOPIC, scan_reliability='reliable',
                 scan_depth=1):
        super().__init__(node_name + '_harness')

        self.node_name = node_name
        self.service_name = '/' + node_name + '/global_localisation'
        self.map_topic = map_topic
        self.scan_topic = scan_topic

        self.hypotheses = []
        self.map_scans = []
        self.poses = []
        self.top_hypotheses = []
        self.transforms = []

        collecting = QoSProfile(depth=50)
        collecting.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(PoseWithCovarianceStamped, output_pose_topic,
                                 self.poses.append, collecting)
        self.create_subscription(TFMessage, '/tf', self.transforms.append,
                                 collecting)

        if hypotheses_topic:
            self.create_subscription(PoseArray, hypotheses_topic,
                                     self.hypotheses.append, collecting)

        if top_hypotheses_topic:
            self.create_subscription(PoseArray, top_hypotheses_topic,
                                     self.top_hypotheses.append, collecting)

        if map_scan_topic:
            self.create_subscription(LaserScan, map_scan_topic,
                                     self.map_scans.append, collecting)

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_publisher = self.create_publisher(OccupancyGrid, map_topic,
                                                   latched)
        self.static_transform_publisher = self.create_publisher(
            TFMessage, '/tf_static', latched)
        scanning = QoSProfile(depth=scan_depth)
        scanning.reliability = (
            ReliabilityPolicy.RELIABLE if scan_reliability == 'reliable'
            else ReliabilityPolicy.BEST_EFFORT)

        self.scan_publisher = self.create_publisher(
            LaserScan, scan_topic, scanning)

        self.transform_publisher = self.create_publisher(
            TFMessage, '/tf', 100)

        self.static_transforms = []

        self.client = self.create_client(Trigger, self.service_name)

    def call(self, timeout=SERVICE_TIMEOUT):
        """Ask for one localisation and return the outcome it answers with."""
        self.wait_for_service(timeout)

        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        assert future.result() is not None, \
            '%s did not answer within %g seconds' % (self.service_name, timeout)

        return future.result()

    def call_twice(self, delay, timeout=SERVICE_TIMEOUT):
        """
        Ask for two localisations delay seconds apart and return both outcomes.

        The second call is sent while the first is still being worked on, which
        is the only way to ask what a node does about a run it has not finished.
        """
        self.wait_for_service(timeout)

        first = self.client.call_async(Trigger.Request())
        self.spin(delay)
        second = self.client.call_async(Trigger.Request())

        deadline = time.monotonic() + timeout

        while not (first.done() and second.done()):
            assert time.monotonic() < deadline, \
                '%s answered %d of the two calls within %g seconds' % (
                    self.service_name, first.done() + second.done(), timeout)
            rclpy.spin_once(self, timeout_sec=0.05)

        return first.result(), second.result()

    def call_while(self, meanwhile, delay, timeout=SERVICE_TIMEOUT):
        """
        Ask for one localisation and do something else part way through it.

        The something else happens delay seconds after the request goes out and
        while the run is still going, which is how a case says what the world did
        during a run rather than only before it.
        """
        self.wait_for_service(timeout)

        answer = self.client.call_async(Trigger.Request())
        self.spin(delay)

        meanwhile()

        deadline = time.monotonic() + timeout

        while not answer.done():
            assert time.monotonic() < deadline, \
                '%s did not answer within %g seconds' % (self.service_name,
                                                         timeout)
            rclpy.spin_once(self, timeout_sec=0.05)

        return answer.result()

    def clear(self):
        """Forget everything collected so far."""
        self.hypotheses.clear()
        self.map_scans.clear()
        self.poses.clear()
        self.top_hypotheses.clear()
        self.transforms.clear()

    def publish_dynamic_transform(self, frame_id, child_frame_id, placement,
                                  stamp):
        """
        Put one transform into the tree as a moving joint's would arrive.

        A transform published here expires: the tree keeps a bounded history, so
        a stamp far enough behind the newest one it holds is a stamp it can no
        longer answer for. That is what a robot's odometry transform is like, and
        what a static one is not.
        """
        self.transform_publisher.publish(TFMessage(transforms=[
            transform_message(frame_id, child_frame_id, placement, stamp)]))

    def publish_map(self, grid, stamp=MAP_STAMP, frame_id=GLOBAL_FRAME):
        """Send one map to the node, latched as a map server publishes one."""
        self.map_publisher.publish(build_map(stamp, grid, frame_id))

    def publish_odometry_placement(self, placement=ROBOT_IN_ODOMETRY):
        """Put the robot's place in the odometry frame into the tree, statically."""
        self.publish_transform(ODOM_FRAME, ROBOT_FRAME, placement)

    def publish_scan(self, ranges, stamp=SCAN_STAMP, frame_id=SENSOR_FRAME,
                     first_ray_angle=FIRST_RAY_ANGLE):
        """Send one scan to the node."""
        self.scan_publisher.publish(
            build_scan(stamp, ranges, frame_id, first_ray_angle))

    def publish_sensor_placement(self, placement=SENSOR_ON_ROBOT):
        """Put where the sensor sits on the robot into the transform tree."""
        self.publish_transform(ROBOT_FRAME, SENSOR_FRAME, placement)

    def publish_transform(self, frame_id, child_frame_id, placement):
        """
        Put one static transform into the tree, beside any already there.

        Every static transform published so far goes out again with it, in one
        message, which is what a static broadcaster does and what makes a single
        latched message enough for whoever subscribes next.
        """
        message = transform_message(frame_id, child_frame_id, placement, (0, 0))

        self.static_transforms = [
            transform for transform in self.static_transforms
            if transform.child_frame_id != child_frame_id] + [message]

        self.static_transform_publisher.publish(
            TFMessage(transforms=self.static_transforms))

    def spin(self, seconds):
        """Process callbacks for a while."""
        deadline = time.monotonic() + seconds

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_connection(self, subscribers=1, timeout=30.0):
        """
        Block until that many nodes have subscribed to everything it drives.

        The transform tree is waited on as well as the map and the scan. A
        static transform is published once and kept, so one published before
        anybody had subscribed is a transform nobody ever receives.
        """
        publishers = {
            self.map_topic: self.map_publisher,
            self.scan_topic: self.scan_publisher,
            '/tf_static': self.static_transform_publisher,
        }
        deadline = time.monotonic() + timeout

        while True:
            subscribed = {topic: publisher.get_subscription_count()
                          for topic, publisher in publishers.items()}

            if all(count >= subscribers for count in subscribed.values()):
                return

            assert time.monotonic() < deadline, \
                'waited %g seconds for %d subscriber each on the topics the ' \
                'node is driven through, and found %s, so the node is either ' \
                'not running or listening elsewhere' % (
                    timeout, subscribers, subscribed)

            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_service(self, timeout=SERVICE_TIMEOUT):
        """Block until the node offers its service."""
        assert self.client.wait_for_service(timeout_sec=timeout), \
            '%s never appeared, so the node offers no way to ask for a ' \
            'localisation' % self.service_name
