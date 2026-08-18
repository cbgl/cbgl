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
What a freshly launched node does, which is nothing until it is asked.

Launching puts the node in stand-by. It subscribes, it offers its service, and
it holds whatever arrives without touching it: a global localisation costs
seconds of a machine's whole attention, so it happens when a caller asks for it
and at no other time.

The cases run in the order their names give, because the last of them asks for a
localisation and a node that has performed one is no longer a node that has
never performed one.

**The map subscription is latched.** A map server publishes its map once and
keeps it, so a subscription that is not durable receives nothing at all from a
map server that was already running. That is asserted here, on the subscription
itself, rather than left to be discovered by a node that waits forever for a map
somebody already published.
"""

import time
import unittest

from cbgl_test_support import (
    Harness,
    keep_alive,
    node_parameters,
    read_grid,
    read_scans,
)
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


@pytest.mark.launch_test
def generate_test_description():
    """Launch one node, configured as this package ships it."""
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


class TestStartupContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.grid = read_grid()
        cls.scan = read_scans()[0]

        cls.harness = Harness()
        cls.harness.wait_for_connection()
        cls.harness.publish_sensor_placement()
        cls.harness.publish_map(cls.grid)
        cls.harness.spin(1.0)

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

    def test_1_the_service_is_offered_and_both_inputs_are_subscribed(self):
        self.harness.wait_for_service(timeout=30.0)

        self.assertIsNotNone(self.endpoint_on('/scan'))
        self.assertIsNotNone(self.endpoint_on('/map'))

    def test_2_the_map_is_taken_latched_and_the_scan_reliably(self):
        map_subscription = self.endpoint_on('/map')

        self.assertEqual(map_subscription.qos_profile.durability,
                         DurabilityPolicy.TRANSIENT_LOCAL,
                         'a map latched before this node started would never '
                         'arrive, and the node would wait for a map that has '
                         'already been published')
        self.assertEqual(map_subscription.qos_profile.depth, 1)

        scan_subscription = self.endpoint_on('/scan')

        self.assertEqual(scan_subscription.qos_profile.reliability,
                         ReliabilityPolicy.RELIABLE)
        self.assertEqual(scan_subscription.qos_profile.depth, 1)

    def test_3_nothing_is_computed_until_the_service_is_called(self):
        self.harness.clear()
        self.harness.publish_scan(self.scan)
        self.harness.spin(3.0)

        self.assertEqual(self.harness.poses, [],
                         'the node answered without being asked. A map and a '
                         'scan in hand are not a request for a localisation')
        self.assertEqual(self.harness.transforms, [],
                         'the node broadcast a transform without being asked')

        response = self.harness.call()

        self.assertTrue(response.success, response.message)
        self.assertTrue(response.message,
                        'the outcome carries no sentence, so a caller that '
                        'wanted to log what happened has nothing to log')

        self.harness.spin(1.0)

        self.assertEqual(len(self.harness.poses), 1,
                         'the call produced %d poses, so what the two '
                         'assertions above found missing was not the call'
                         % len(self.harness.poses))


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_the_node_exited_cleanly(self, proc_info, cbgl):
        launch_testing.asserts.assertExitCodes(proc_info, process=cbgl)
