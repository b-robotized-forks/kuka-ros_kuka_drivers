# Copyright 2026 KUKA Hungaria Kft.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Drives come on after an error stop's message holds them off, without a reconfigure."""

import os
import re
import sys
import unittest

import launch_testing.asserts
import launch_testing.markers
import pytest
from controller_manager_msgs.srv import SetHardwareComponentState
from lifecycle_msgs.msg import State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rsi_fault_support import (  # noqa: E402
    COMPONENT,
    OP_MODE_EXT,
    LogCursor,
    RsiFaultTestCase,
    fault_launch_description,
)

DRIVES_POWERED_LINE = re.compile(r"Drives successfully powered on after (\d+) ms")
MESSAGE_RESET_LINE = "resetting the controller's messages"
FAKE_RESET_LINE = "mxA status fake: message reset requested"
DRIVES_FAILED_LINE = "Activation failed: drives not powered"
FAKE_CLEARED_LINE = "mxA status fake: messages reset, error message cleared"
STOP_COMPLETE_LINE = "Stop complete"
STARTING_LINE = "Starting joint position control at 4 ms"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return fault_launch_description(respawn_simulator=True)


class TestDrivesOnAfterErrorStop(RsiFaultTestCase):
    def request_active(self):
        request = SetHardwareComponentState.Request()
        request.name = COMPONENT
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        client = self.node.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=30), "no component state service")
        return client.call_async(request)

    def activate(self, log, respawned):
        future = self.request_active()
        if not respawned:
            self.wait_for_simulator_respawn(log)
        self.assertTrue(log.wait_for(STARTING_LINE, timeout=10), "no RSI start")
        self.set_simulator_fault("fault_dropped_packet_count", 0)
        self.assertTrue(self._wait_future(future, 60).ok, "activate refused")

    def test_drives_on_after_error_stop(self, proc_output):
        self.ensure_component_state(State.PRIMARY_STATE_INACTIVE)
        self.publish_is_configured()
        self.inject(f"operation_mode={OP_MODE_EXT} motion_possible=1")
        log = LogCursor(proc_output)
        self.activate(log, respawned=True)
        self.assertIsNotNone(log.wait_for_match(DRIVES_POWERED_LINE, timeout=1), "not powered")
        self.assertEqual(
            log.matches(re.compile(MESSAGE_RESET_LINE)), [], "reset with no stuck edge"
        )

        log = LogCursor(proc_output)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")
        self.assertEqual(self.state_value("drives_powered"), 0.0)

        # With the guard open no edge can power the drives, so the messages are left alone.
        self.wait_for_simulator_respawn(log)
        log = LogCursor(proc_output)
        self.inject("guard_stop=1 error_message=1")
        self.assertFalse(self.set_component_state(State.PRIMARY_STATE_ACTIVE), "activated")
        self.assertTrue(log.wait_for(DRIVES_FAILED_LINE, timeout=1), "no drives-on failure")
        self.assertEqual(log.matches(re.compile(MESSAGE_RESET_LINE)), [], "reset with guard open")
        self.assertEqual(
            log.matches(re.compile(FAKE_RESET_LINE)), [], "reset sent with guard open"
        )

        # The KRC ignores DRIVES_ON until the error stop's message is reset.
        log = LogCursor(proc_output)
        self.inject("guard_stop=0")
        self.activate(log, respawned=True)
        self.assertTrue(log.wait_for(MESSAGE_RESET_LINE, timeout=1), "no message reset")
        self.assertTrue(log.wait_for(FAKE_CLEARED_LINE, timeout=1), "message never cleared")
        self.assertIsNotNone(log.wait_for_match(DRIVES_POWERED_LINE, timeout=1), "not powered")
        self.assertEqual(self.component_state(), State.PRIMARY_STATE_ACTIVE)
        # A slot set published before the activation can still be in flight.
        self._spin_for(0.5)
        self.assert_latched(set())

        log = LogCursor(proc_output)
        self.assertTrue(self.set_component_state(State.PRIMARY_STATE_INACTIVE), "deactivate")
        self.assertTrue(log.wait_for(STOP_COMPLETE_LINE, timeout=5), "stop never completed")
        self.wait_for_simulator_respawn(log)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[launch_testing.asserts.EXIT_OK],
        )
