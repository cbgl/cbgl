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
A misconfigured node dies at once, and says which setting killed it.

A setting outside its permitted range cannot be worked around and must not be
worked around: a run performed with a nonsense density, or against a caster
nobody recognises, is a wrong answer produced confidently. So the node refuses
to come up at all, and the sentence it leaves behind names the setting and the
value it was given, because that sentence is the whole of what an operator
reading a log has to work from.

Every case is launched at once, each as a process of its own with a name of its
own. None of them needs anything from the others, and each dies on its own.

**One of them is the control**, launched with every setting inside its range. It
has to be running still when the pre-shutdown cases end, which is asserted of the
process itself rather than of the code it eventually exits with: a node that came
up and stopped a moment later would exit as cleanly as one that stayed up, and
only one of the two is a node. Without the control the file would pass against a
node that refused every configuration it was ever given, which is not a node that
validates its settings, it is a node that does not start.

**Four of the cases exist because no validator can catch them.**

The whole-number settings are read as signed and used as unsigned. A negative
`da` becomes an enormous positive count that the library's own rule, which asks
only for at least one orientation, cannot tell from a deliberate setting, and the
run would then ask for that many orientations of every hypothesis it drew. `da`
is the one chosen for this because it is the multiplier: the candidates scored are
the hypotheses drawn times `da`, so a value that wrapped there asks for an
allocation no machine has rather than merely a run that takes too long.

A transform lookup timeout that is not a finite number reaches a duration
constructor that has nothing sensible to do with it.

A topic name that is empty names nothing, and the answer would have nowhere to go.

A frame id that is empty, or that begins with a slash, is the one the published
build got wrong: it wrote a slashed frame into the answer it published, which tf2
rejects outright. Refusing it at start-up is what keeps a configuration from
carrying that mistake back in.

**The caster's name is refused by nobody but the node.** The matcher's own
validator, which judges the rest of the matcher's settings, carries no rule for
it, and the library defers to that validator rather than keeping a second copy.
So the parse of the name into a casting method, which happens in the node, is
what has to refuse an unknown one, and it has to refuse it in the same words the
rest do. That is why the caster case is here rather than among the library's own
parameter cases.

**The processes are held in a module of their own rather than asked for one at a
time.** There are eleven of them, and naming each as an argument of each case,
twice over, would be a page of names before the first assertion. They are the
same objects the launch description carries.
"""

import unittest

from cbgl_test_support import keep_alive, node_parameters
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest

CASES = {
    'bad_caster': ({'map_scan_method': 'vanilla'},
                   ('map_scan_method', 'vanilla')),
    'bad_da': ({'da': -1}, ('da', '-1')),
    'bad_dl': ({'dl': -2.5}, ('dl', '-2.5')),
    'bad_frame': ({'global_frame_id': '/map'},
                  ('global_frame_id', '/map')),
    'bad_output_topic': ({'output_pose_topic': ''},
                         ('output_pose_topic', 'must name a topic')),
    'bad_ray_search': ({'ray_search': 'spiral'}, ('ray_search', 'spiral')),
    'bad_reliability': ({'scan_qos_reliability': 'best'},
                        ('scan_qos_reliability', 'best')),
    'bad_threshold': ({'occupancy_threshold': 101},
                      ('occupancy_threshold', '101')),
    'bad_timeout': ({'tf_lookup_timeout': float('inf')},
                    ('tf_lookup_timeout', 'inf')),
    'bad_top_k': ({'top_k_caers': 0}, ('top_k_caers', '0')),
}

CONTROL = 'in_range'
CONTROL_WATCH = 5.0

PROCESSES = {}


def cbgl_node(node_name, **overrides):
    """Return one node, launched with the settings this case is about."""
    return launch_ros.actions.Node(
        package='cbgl',
        executable='cbgl_interface_node',
        name=node_name,
        output='screen',
        parameters=[node_parameters(**overrides)],
    )


@pytest.mark.launch_test
def generate_test_description():
    """Launch one node per refusal, and one that has nothing to refuse."""
    PROCESSES.update({
        name: cbgl_node('cbgl_' + name, **overrides)
        for name, (overrides, _) in CASES.items()
    })

    PROCESSES[CONTROL] = cbgl_node('cbgl_' + CONTROL)

    return (
        launch.LaunchDescription(
            list(PROCESSES.values())
            + [keep_alive(), launch_testing.actions.ReadyToTest()]),
        PROCESSES,
    )


class TestEveryRefusalIsSaidOutLoud(unittest.TestCase):

    def test_every_refusal_names_its_setting_and_the_value_it_was_given(
            self, proc_info, proc_output):
        """
        Say the setting and the value, and then be gone of your own accord.

        Each process is waited on until it has exited before this case returns.
        Without that wait the launch is torn down while the slowest of them is
        still on its way out, it is signalled rather than left to exit, and the
        exit code the later case asserts would be the signal's rather than the
        refusal's.
        """
        for name, (_, expected) in CASES.items():
            for phrase in expected:
                proc_output.assertWaitFor(phrase, process=PROCESSES[name],
                                          timeout=60, stream='stderr')

        for name in CASES:
            proc_info.assertWaitForShutdown(process=PROCESSES[name], timeout=60)

    def test_the_configuration_with_nothing_to_refuse_is_still_running(
            self, proc_info):
        """
        Be running still, once every misconfigured node has gone.

        This runs after the case above, so every process that had something to
        refuse has already exited by the time it is asked. What is asserted is
        that waiting for the control to finish times out, and it is put that way
        round on purpose: asking once whether it has finished yet is a question
        about this instant, and the answer can be no simply because the exit has
        not been recorded yet. Waiting instead makes the whole window count, so a
        control that dies at any point inside it fails here.

        Asserting a code cannot do this job. A node that came up and stopped a
        moment later exits reporting nothing wrong, exactly as one that was still
        serving when the launch took it down, so the two are indistinguishable
        after the fact and have to be told apart while the launch is still up.
        """
        finished = True

        try:
            proc_info.assertWaitForShutdown(process=PROCESSES[CONTROL],
                                            timeout=CONTROL_WATCH)
        except AssertionError:
            finished = False

        self.assertFalse(
            finished,
            'the node with every setting inside its range stopped within %g '
            'seconds of every misconfigured one exiting. It refused a '
            'configuration it had nothing to refuse about, or it came up and '
            'stopped, and either way the refusals above prove nothing about the '
            'settings that caused them' % CONTROL_WATCH)


@launch_testing.post_shutdown_test()
class TestEveryRefusalEndsTheProcess(unittest.TestCase):

    def test_a_configuration_inside_every_range_ended_only_by_the_teardown(
            self, proc_info):
        """
        End on the launch's terms rather than on its own.

        The discriminating half of the control is the pre-shutdown case, which
        requires the process to be alive after every refusal has happened. This
        adds what an operator sees afterwards: the node handles the signal the
        teardown sends, so it exits reporting nothing wrong. What is ruled out is
        the one a refusal exits with, so a node that refused this configuration
        late rather than at once still fails here.
        """
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15],
            process=PROCESSES[CONTROL])

    def test_every_refusal_exits_rather_than_aborts(self, proc_info):
        """
        End the process on purpose rather than by aborting.

        What is being ruled out is the abort an uncaught exception leaves, which
        would bury the sentence above under a message about an exception nobody
        caught, and that is not what an operator needs to read.
        """
        for name in CASES:
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=[1], process=PROCESSES[name])
