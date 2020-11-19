"""
license BSD 3-clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft

Unit-tests for the python API of the FollowTarget.
"""

import unittest
import numpy as np

from dg_tutorial_with_turtlesim.follow_target_graph import FollowTarget


class TestFollowTarget(unittest.TestCase):
    def test_create_graph(self):

        try:
            FollowTarget("unit-test-0")
        except:
            self.assertRaises("FollowTarget has raised unexpectedely.")

    def test_set_gains_to_zero_gains(self):

        ctrl = FollowTarget("unit-test-1")
        ctrl.robot_absolute_position_sin.value = np.array([1.0, 1.0])
        ctrl.robot_orientation_sin.value = np.array([0.0])
        ctrl.absolute_target_position_sin.value = np.array([2.0, 2.0])

        ctrl.set_gains(0.0, 0.0)

        ctrl.control_sout.recompute(1)

        np.testing.assert_equal(ctrl.control_sout.value, np.array([0.0, 0.0]))

    def test_set_gains_to_zero_error(self):

        ctrl = FollowTarget("unit-test-2")
        ctrl.robot_absolute_position_sin.value = np.array([1.0, 1.0])
        ctrl.robot_orientation_sin.value = np.array([0.0])
        ctrl.absolute_target_position_sin.value = np.array([1.0, 1.0])

        ctrl.set_gains(5.0, 2.0)

        ctrl.control_sout.recompute(1)

        np.testing.assert_equal(ctrl.control_sout.value, np.array([0.0, 0.0]))

    def test_evaluate_graph(self):

        ctrl = FollowTarget("unit-test-3")
        ctrl.robot_absolute_position_sin.value = np.array([1.0, 1.0])
        ctrl.robot_orientation_sin.value = np.array([0.0])
        ctrl.absolute_target_position_sin.value = np.array([2.0, 2.0])

        ctrl.set_gains(5.0, 2.0)

        ctrl.control_sout.recompute(1)

        np.testing.assert_almost_equal(
            ctrl.control_sout.value, np.array([5.0, 3.926991]),
            decimal=5
        )
