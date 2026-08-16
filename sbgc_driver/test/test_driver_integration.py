# Copyright 2026 Yousef Hussein
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Drive the real node against a simulated SimpleBGC board on a pty.

The board is the upstream project's own simulator, so this exercises real
serial framing, the lifecycle transitions and the safety gates end to end --
no mock that agrees with whatever the code happens to do.

These assert the properties that matter and cannot be checked without a node:
that telemetry becomes the right ROS messages, that the watchdog closes when a
publisher stops, and that a board fault reaches diagnostics.
"""

import os
import sys
import time
import unittest

from control_msgs.msg import JointJog

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState

import pytest

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sbgc_interfaces.msg import GimbalStatus

from sensor_msgs.msg import BatteryState, JointState

from std_srvs.srv import SetBool, Trigger

# The upstream project's board simulator is not an installed package; it lives
# in the vendored tree beside the protocol it speaks.
sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), '..', '..', 'vendor', 'simplebgc32-control', 'test'))

from sbgc_sim import Board  # noqa: E402,I100

SBGC_ERR_EMERGENCY_STOP = 1 << 11

_board = None


@pytest.mark.launch_test
def generate_test_description():
    """Start a simulated board, then point the driver at its pty."""
    global _board
    # Board pitch is positive-downward, so +30 here must arrive as -30 in ROS.
    _board = Board(motors_on=True, angles=(0.0, 30.0, 45.0))

    driver = launch_ros.actions.Node(
        package='sbgc_driver',
        executable='sbgc_driver_node',
        name='sbgc_driver',
        output='screen',
        parameters=[{
            'port': _board.device,
            'allow_control': True,
            'start_armed': True,
            'command_timeout': 0.5,
            'status_rate_hz': 20.0,
        }],
    )

    return launch.LaunchDescription([
        driver,
        launch_testing.actions.ReadyToTest(),
    ]), {'driver': driver}


class Helper(Node):
    """A subscriber that keeps the latest message of each topic it watches."""

    def __init__(self):
        super().__init__('sbgc_integration_helper')
        self.joint_states = None
        self.status = None
        self.battery = None
        self.diagnostics = None

        self.create_subscription(
            JointState, '/joint_states', self._set('joint_states'),
            qos_profile_sensor_data)
        self.create_subscription(
            GimbalStatus, '/sbgc_driver/status', self._set('status'), 10)
        self.create_subscription(
            BatteryState, '/sbgc_driver/battery', self._set('battery'), 10)
        self.create_subscription(
            DiagnosticArray, '/diagnostics', self._set('diagnostics'), 10)

        self.jog = self.create_publisher(JointJog, '/sbgc_driver/joint_jog', 10)

    def _set(self, name):
        def cb(msg):
            setattr(self, name, msg)
        return cb

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def await_field(self, attr, predicate, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            value = getattr(self, attr)
            if value is not None and predicate(value):
                return value
        raise AssertionError(f'timed out waiting for {what}')

    def transition(self, transition_id):
        client = self.create_client(ChangeState, '/sbgc_driver/change_state')
        assert client.wait_for_service(timeout_sec=10.0), 'no lifecycle service'
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        assert future.result() is not None and future.result().success, \
            f'transition {transition_id} failed'

    def call(self, srv_type, name, request, timeout=10.0):
        client = self.create_client(srv_type, name)
        assert client.wait_for_service(timeout_sec=timeout), f'no service {name}'
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        assert future.result() is not None, f'{name} did not answer'
        return future.result()


class TestGimbalDriver(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Helper()
        cls.node.transition(Transition.TRANSITION_CONFIGURE)
        cls.node.transition(Transition.TRANSITION_ACTIVATE)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        if _board is not None:
            _board.close()

    def test_board_angles_become_joint_states_in_ros_convention(self):
        js = self.node.await_field(
            'joint_states', lambda m: len(m.position) == 3, 15.0, 'joint_states')

        self.assertEqual(
            list(js.name),
            ['gimbal_roll_joint', 'gimbal_pitch_joint', 'gimbal_yaw_joint'])

        # The board reports pitch positive-DOWN. A driver that passed the sign
        # through would tilt the camera the wrong way and apply the asymmetric
        # travel limits to the wrong direction.
        self.assertAlmostEqual(js.position[1], -0.5236, places=2,
                               msg='board pitch +30 must arrive as ROS -30')
        self.assertAlmostEqual(js.position[2], 0.7854, places=2)

    def test_the_link_reports_itself_healthy(self):
        status = self.node.await_field(
            'status', lambda m: m.board_responding, 15.0, 'a responding board')
        self.assertTrue(status.link_open)
        self.assertFalse(status.simulated)
        self.assertGreater(status.frames_received, 0)
        self.assertTrue(status.motors_on)

    def test_battery_marks_unmeasured_fields_as_nan_not_zero(self):
        # Zero would read as a flat battery; the message defines NaN as
        # "not measured".
        bat = self.node.await_field(
            'battery', lambda m: m.voltage > 0.0, 15.0, 'a battery reading')
        self.assertAlmostEqual(bat.voltage, 12.0, places=1)
        self.assertTrue(bat.present)
        for field in ('current', 'charge', 'capacity', 'percentage', 'temperature'):
            self.assertNotEqual(
                getattr(bat, field), 0.0, msg=f'{field} must be NaN, not zero')

    def test_the_watchdog_closes_when_a_publisher_stops(self):
        # This is the fail-closed guarantee: a publisher that dies stops the
        # camera rather than leaving it running.
        msg = JointJog()
        msg.joint_names = ['gimbal_yaw_joint']
        msg.velocities = [0.2]

        end = time.time() + 1.5
        while time.time() < end:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.node.jog.publish(msg)
            self.node.spin_for(0.05)

        moving = self.node.await_field(
            'status', lambda m: not m.command_timeout, 3.0,
            'the driver to accept a command')
        self.assertFalse(moving.command_timeout)

        # Now stop publishing and let the watchdog expire.
        held = self.node.await_field(
            'status', lambda m: m.command_timeout, 5.0,
            'the watchdog to close after the publisher stopped')
        self.assertTrue(held.command_timeout)

    def test_stop_works_while_disarmed(self):
        # The gate exists to prevent unwanted motion; refusing to stop would
        # invert its purpose.
        self.node.call(SetBool, '/sbgc_driver/arm', SetBool.Request(data=False))
        result = self.node.call(Trigger, '/sbgc_driver/stop', Trigger.Request())
        self.assertTrue(result.success, result.message)
        self.node.call(SetBool, '/sbgc_driver/arm', SetBool.Request(data=True))

    def test_a_board_fault_reaches_status_and_diagnostics(self):
        # The board deprecated its one-byte error code, so a current firmware
        # can report a fault with that byte still reading zero.
        _board.set(system_error=SBGC_ERR_EMERGENCY_STOP)
        try:
            status = self.node.await_field(
                'status', lambda m: m.system_error != 0, 10.0,
                'the fault to reach GimbalStatus')
            self.assertEqual(status.deprecated_error_code, 0)
            self.assertEqual(status.system_error_name, 'emergency stop')

            def has_error(msg):
                return any(
                    s.name.endswith('gimbal') and s.level == DiagnosticStatus.ERROR
                    for s in msg.status)

            self.node.await_field(
                'diagnostics', has_error, 10.0,
                'the fault to reach /diagnostics')
        finally:
            _board.set(system_error=0)

    def test_a_silent_board_is_reported_rather_than_frozen(self):
        # A board whose transmit path dies leaves the last angle frozen and
        # perfectly plausible. The driver must say so instead of continuing to
        # present it as current.
        _board.set(answer_realtime=False)
        try:
            stale = self.node.await_field(
                'status', lambda m: not m.board_responding, 10.0,
                'the driver to notice a silent board')
            self.assertGreater(
                stale.last_frame_age.sec + stale.last_frame_age.nanosec * 1e-9, 0.4)
        finally:
            _board.set(answer_realtime=True)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_the_node_exits_cleanly(self, proc_info, driver):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15], process=driver)
