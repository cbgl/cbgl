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
Every run the node itself refuses, each provoked by its own missing precondition.

Five things the node decides on its own, before the library is asked anything:
that no map has arrived, that no scan has arrived, that the transform tree does
not say where the sensor sits on the robot, that the scan it holds does not span
a turn the way the matcher's ray contract requires, and that a run is already
under way. The library's own refusals, which are about the content of a map or a
scan rather than about what the node has in hand, are reached by the library's
own cases.

**Each case starves the node of one thing and of nothing else.** That is the
whole design of this file, and it is not free. Reaching the right sentence is not
the same as reaching it for the right reason: the node tests its preconditions in
an order, so a node starved of everything answers about whichever it happens to
test first, and a case written that way would keep passing after the precondition
it was meant to be about had stopped being checked at all. So every case here
gives the node everything except the one thing it is about, and then asserts that
the sentence names that thing and does not name the others.

**One node per case, each driven through topics of its own.** A map and a scan
are ordinary topics, so two nodes in one launch cannot be given different ones
unless they listen in different places. Each node here is therefore configured
with a map topic, a scan topic and an answer topic that nothing else in the
launch shares, which is what lets one node have a map and no scan while another
has a scan and no map, at the same time, in the same process.

**The transform tree cannot be divided that way.** There is one tree, and a
static transform published into it is there for everybody and for good. So the
case about a missing placement does not withhold the tree: it publishes a scan
naming a frame the tree does not carry, while the tree does carry the placement
the other three nodes use. The map is present, the scan is present, the tree is
populated, and the one thing missing is the transform for that one frame.

**And each case restores its own precondition and requires the refusal to
stop.** A refusal that cannot be made to go away proves nothing about what caused
it. The map arrives and the same node answers; the scan arrives and the same node
answers; the missing placement is published and the same node answers; a scan
that does span the turn properly arrives and the same node answers; the run under
way finishes and the same node answers the next request. Each of those second
halves publishes exactly one thing, so what made the difference is not in
question.
"""

import unittest

from cbgl_test_support import (
    Harness,
    keep_alive,
    node_parameters,
    read_grid,
    read_scans,
    ROBOT_FRAME,
    SENSOR_ON_ROBOT,
)
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy

ASKEW = 'cbgl_askew'
BUSY = 'cbgl_busy'
MAPLESS = 'cbgl_mapless'
SCANLESS = 'cbgl_scanless'
UNPLACED = 'cbgl_unplaced'

UNPLACED_SENSOR_FRAME = 'unplaced_sensor_frame'

OVERLAP_DELAY = 0.5


def cbgl_node(node_name, **overrides):
    """Return one node, driven through topics nothing else in the launch shares."""
    return launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name=node_name,
        output='screen',
        parameters=[node_parameters(**dict(topics_of(node_name), **overrides))],
    )


@pytest.mark.launch_test
def generate_test_description():
    """Launch one node per refusal, each starved of one thing and no more."""
    nodes = {
        ASKEW: cbgl_node(ASKEW),
        MAPLESS: cbgl_node(MAPLESS),
        SCANLESS: cbgl_node(SCANLESS),
        UNPLACED: cbgl_node(UNPLACED),
        BUSY: cbgl_node(BUSY, dl=150.0, top_k_caers=1500, num_threads=1),
    }

    return (
        launch.LaunchDescription(
            list(nodes.values())
            + [keep_alive(), launch_testing.actions.ReadyToTest()]),
        nodes,
    )


def harness_for(node_name):
    """Return a harness wired to one node's own topics."""
    return Harness(node_name=node_name, **topics_of(node_name))


def topics_of(node_name):
    """Return the map, scan and answer topics one node alone is driven through."""
    return {
        'map_topic': '/' + node_name + '/map',
        'scan_topic': '/' + node_name + '/scan',
        'output_pose_topic': '/' + node_name + '/pose',
    }


class TestEveryRefusalTheNodeDecides(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

        cls.grid = read_grid()
        cls.scan = read_scans()[0]

        cls.askew = harness_for(ASKEW)
        cls.busy = harness_for(BUSY)
        cls.mapless = harness_for(MAPLESS)
        cls.scanless = harness_for(SCANLESS)
        cls.unplaced = harness_for(UNPLACED)

        for harness in cls.every_harness():
            harness.wait_for_connection()
            harness.publish_sensor_placement()

        cls.askew.publish_map(cls.grid)
        cls.busy.publish_map(cls.grid)
        cls.mapless.publish_scan(cls.scan)
        cls.scanless.publish_map(cls.grid)
        cls.unplaced.publish_map(cls.grid)

        for harness in cls.every_harness():
            harness.spin(1.0)

        cls.askew.publish_scan(cls.scan, first_ray_angle=0.0)
        cls.busy.publish_scan(cls.scan)
        cls.unplaced.publish_scan(cls.scan, frame_id=UNPLACED_SENSOR_FRAME)

        for harness in (cls.askew, cls.busy, cls.unplaced):
            harness.spin(0.5)

    @classmethod
    def every_harness(cls):
        """Return every harness, one per node this launch carries."""
        return (cls.askew, cls.busy, cls.mapless, cls.scanless, cls.unplaced)

    @classmethod
    def tearDownClass(cls):
        for harness in cls.every_harness():
            harness.destroy_node()

        rclpy.shutdown()

    def assert_answered(self, harness):
        """Require a run to be performed and its answer to be published."""
        harness.clear()

        outcome = harness.call()
        harness.spin(1.0)

        self.assertTrue(
            outcome.success,
            'the precondition this case is about was restored and the run was '
            'refused all the same, with: %s. Either something else was missing '
            'all along, in which case the refusal above was not about what this '
            'case says it was, or restoring it is not enough to make the node '
            'answer' % outcome.message)

        self.assertEqual(
            len(harness.poses), 1,
            'the run reported success and published %d poses on %s'
            % (len(harness.poses), harness.node_name))

    def assert_refused(self, harness, about):
        """Require a run to be refused with a sentence, and no pose published."""
        harness.clear()

        outcome = harness.call()
        harness.spin(0.5)

        self.assertFalse(
            outcome.success,
            'the run was performed where it had to be refused for %s: %s'
            % (about, outcome.message))

        self.assertTrue(
            outcome.message,
            'the run was refused without a word about why, which leaves a '
            'caller with nothing to report and nothing to correct')

        self.assertEqual(harness.poses, [],
                         'a pose was published for a run that was refused')

        return outcome.message

    def assert_says_nothing_of(self, message, *preconditions):
        """Require a refusal to be silent about the preconditions that were met."""
        for phrase in preconditions:
            self.assertNotIn(
                phrase, message.lower(),
                'the refusal mentions %r, and that precondition was met before '
                'the run was asked for. Either the node refused for a reason '
                'this case did not intend, or its sentences do not distinguish '
                'the reasons: %s' % (phrase, message))

    def test_a_run_before_any_map_is_refused_and_answered_once_one_arrives(self):
        """
        Refuse for want of a map, with everything else in hand.

        This node has a scan, and the tree carries the placement for the frame
        that scan names, so the map is the only thing it lacks. The sentence has
        to name this node's own map topic rather than the default, since it is
        the topic an operator would go and look at.
        """
        message = self.assert_refused(self.mapless, 'want of a map')

        self.assertIn('no map', message.lower(), message)
        self.assertIn(
            topics_of(MAPLESS)['map_topic'], message,
            'the refusal does not say which topic the map was expected on, '
            'which is the one thing an operator needs in order to look: %s'
            % message)
        self.assert_says_nothing_of(message, 'no scan', 'transform tree')

        self.mapless.publish_map(self.grid)
        self.mapless.spin(1.0)

        self.assert_answered(self.mapless)

    def test_a_run_before_any_scan_is_refused_and_answered_once_one_arrives(self):
        """
        Refuse for want of a scan, with everything else in hand.

        The mirror of the case above: this node has a map, and the tree carries
        the placement the scan it is about to receive will need, so the scan is
        the only thing it lacks.
        """
        message = self.assert_refused(self.scanless, 'want of a scan')

        self.assertIn('no scan', message.lower(), message)
        self.assertIn(
            topics_of(SCANLESS)['scan_topic'], message,
            'the refusal does not say which topic the scan was expected on: %s'
            % message)
        self.assert_says_nothing_of(message, 'no map', 'transform tree')

        self.scanless.publish_scan(self.scan)
        self.scanless.spin(0.5)

        self.assert_answered(self.scanless)

    def test_a_run_without_the_sensors_placement_is_refused_and_answered_once_it_is_there(self):
        """
        Refuse for want of one transform, not for want of anything else.

        This is the case most easily written to pass for the wrong reason. The
        node has a map and a scan, and the tree it is asked to search is not
        empty: it carries the placement the other three nodes in this launch use.
        What is withheld is the placement for the frame this node's own scan
        names, and nothing else, so the refusal can only be about the transform.
        The two frame names are what is asserted of the sentence. The node was
        told one of them and read the other off the scan message, so a sentence
        carrying both is a sentence an operator can act on.
        """
        message = self.assert_refused(self.unplaced, 'want of a placement')

        self.assertIn(ROBOT_FRAME, message,
                      'the refusal does not name the robot frame it looked '
                      'for: %s' % message)
        self.assertIn(UNPLACED_SENSOR_FRAME, message,
                      'the refusal does not name the sensor frame the scan '
                      'itself carried: %s' % message)
        self.assert_says_nothing_of(message, 'no map', 'no scan')

        self.unplaced.publish_transform(ROBOT_FRAME, UNPLACED_SENSOR_FRAME,
                                        SENSOR_ON_ROBOT)
        self.unplaced.spin(1.0)

        self.assert_answered(self.unplaced)

    def test_a_scan_that_does_not_span_the_turn_is_refused_and_answered_once_one_does(self):
        """
        Refuse a scan whose rays do not begin where the ray contract puts them.

        Measured ray zero sits at minus pi from the sensor's heading and the rays
        span exactly one turn. A scan that spans a turn from zero instead is a
        legal scan and a different one: read as though it began at minus pi it
        describes the room turned by half a turn, and what a caller gets back is
        not a refusal but a confident answer that is wrong by that much. Silently.
        So the refusal has to name the field it read and the value that field
        carried, since the scan is nobody's fault and its publisher is who has to
        be told.

        The node has a map, the tree carries the placement for the frame this
        scan names, and the scan itself is present, so the angles are the only
        thing wrong with it. Then a scan that does begin at minus pi is published
        to the same node and it answers, which is what shows the refusal was
        about the angles rather than about the scan being absent.
        """
        message = self.assert_refused(self.askew, 'a scan that is not panoramic')

        self.assertIn(
            'angle_min', message,
            'the refusal does not name the field it read, so whoever publishes '
            'this scan cannot tell what about it was refused: %s' % message)
        self.assert_says_nothing_of(message, 'no map', 'no scan',
                                    'transform tree')

        self.askew.publish_scan(self.scan)
        self.askew.spin(0.5)

        self.assert_answered(self.askew)

    def test_a_second_run_while_one_is_under_way_is_refused_and_answered_once_it_ends(self):
        """
        Refuse the second of two overlapping requests, then answer the next one.

        **The overlap is sized from a measurement rather than hoped for.** Timed
        through this very service in the container this suite runs in: on the
        fixture room at the density this package ships, a run answers in 0.345 s
        the first time and 0.047 s warm, over 2656 candidates. This node is
        launched at ten times that density, which is the 834 hypotheses the
        shipped density would draw from a room ten times this one, and it refines
        1500 survivors of them on a single thread. That run was measured at 9.46 s
        cold and 8.85 s warm. The second request is sent 0.5 s in, which is a
        margin of about seventeen times against the warmer figure. The margin is
        a ratio, so a faster machine shortens the run and the delay together and
        a loaded one lengthens both; it is not a race that a quick container
        wins.

        Which of the two is refused is not asserted, only that exactly one was.
        Both requests are in flight at once and either could be the one the node
        takes up first; what matters is that it takes up one and refuses the
        other, and that only one pose is published.
        """
        self.busy.clear()

        outcomes = self.busy.call_twice(OVERLAP_DELAY)

        performed = [outcome for outcome in outcomes if outcome.success]
        refused = [outcome for outcome in outcomes if not outcome.success]

        self.assertEqual(
            len(performed), 1,
            'of two overlapping requests %d were performed. The sentences were: '
            '%s' % (len(performed), [outcome.message for outcome in outcomes]))

        self.assertIn(
            'already', refused[0].message.lower(),
            'the refusal does not say that a run was already under way, so a '
            'caller cannot tell it from a refusal it ought to act on: %s'
            % refused[0].message)
        self.assert_says_nothing_of(refused[0].message, 'no map', 'no scan',
                                    'transform tree')

        self.busy.spin(1.0)

        self.assertEqual(
            len(self.busy.poses), 1,
            'the two requests published %d poses, so the second was queued and '
            'run rather than refused' % len(self.busy.poses))

        self.assert_answered(self.busy)


@launch_testing.post_shutdown_test()
class TestNodeShutdown(unittest.TestCase):

    def test_the_nodes_exited_cleanly(self, proc_info, cbgl_askew, cbgl_busy,
                                      cbgl_mapless, cbgl_scanless,
                                      cbgl_unplaced):
        for process in (cbgl_askew, cbgl_busy, cbgl_mapless, cbgl_scanless,
                        cbgl_unplaced):
            launch_testing.asserts.assertExitCodes(proc_info, process=process)
