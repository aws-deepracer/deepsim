#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
from typing import Any, Callable
from unittest import TestCase
from unittest.mock import patch, MagicMock, call
import inspect
import time

import deepsim.sim_trackers.constants as consts
from deepsim.sim_trackers.tracker_manager import TrackerManager
from deepsim.deepsim import DeepSim


myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.sim_trackers.tracker_manager.rospy")
class TrackerManagerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_step_start(self, rospy_mock):
        with patch("deepsim.sim_trackers.tracker_manager.threading.Thread") as thread_mock:
            tracker_manager = TrackerManager(is_singleton=False)
            tracker_manager.start()
            assert tracker_manager._last_time is None
            assert not tracker_manager._callback_event.isSet()
            assert not tracker_manager._should_stop_update
            assert tracker_manager._update_loop_thread == thread_mock.return_value
            thread_mock.return_value.start.assert_called_once()

    def test_step_stop(self, rospy_mock):
        with patch("deepsim.sim_trackers.tracker_manager.threading.Thread") as thread_mock:
            tracker_manager = TrackerManager(is_singleton=False)
            tracker_manager.start()
            tracker_manager.stop()
            assert tracker_manager._should_stop_update
            assert tracker_manager._callback_event.isSet()
            thread_mock.return_value.join.assert_called_once()
            assert tracker_manager._update_loop_thread is None

    def test_step_update_loop(self, rospy_mock):
        tracker_manager = TrackerManager(is_singleton=False)
        tracker_manager.start()

        tracker_mock = MagicMock()
        tracker_manager.add(tracker_mock)
        assert tracker_mock in tracker_manager._tracker_map[consts.TrackerPriority.NORMAL]

        tracker_mock2 = MagicMock()
        tracker_manager.add(tracker_mock2, consts.TrackerPriority.HIGH)
        assert tracker_mock2 in tracker_manager._tracker_map[consts.TrackerPriority.HIGH]

        tracker_mock3 = MagicMock()
        tracker_manager.add(tracker_mock3, consts.TrackerPriority.LOW)
        assert tracker_mock3 in tracker_manager._tracker_map[consts.TrackerPriority.LOW]

        sim_time_mock = MagicMock()
        sim_time_mock.clock.secs = 1.0
        sim_time_mock.clock.nsecs = 9000.0
        tracker_manager._sim_time = sim_time_mock
        tracker_manager._callback_event.set()
        while tracker_manager._callback_event.is_set():
            time.sleep(0)
        tracker_manager.stop()
        assert tracker_manager._last_time == sim_time_mock.clock.secs + 1.e-9 * sim_time_mock.clock.nsecs
        tracker_mock.on_update_tracker.assert_called_once()
        tracker_mock2.on_update_tracker.assert_called_once()
        tracker_mock3.on_update_tracker.assert_called_once()

    def test_step_update_loop_skip(self, rospy_mock):
        tracker_manager = TrackerManager(is_singleton=False)
        DeepSim.get_instance().timestep = 3.0
        tracker_manager.start()

        tracker_mock = MagicMock()
        tracker_manager.add(tracker_mock)
        assert tracker_mock in tracker_manager._tracker_map[consts.TrackerPriority.NORMAL]

        tracker_mock2 = MagicMock()
        tracker_manager.add(tracker_mock2, consts.TrackerPriority.HIGH)
        assert tracker_mock2 in tracker_manager._tracker_map[consts.TrackerPriority.HIGH]

        tracker_mock3 = MagicMock()
        tracker_manager.add(tracker_mock3, consts.TrackerPriority.LOW)
        assert tracker_mock3 in tracker_manager._tracker_map[consts.TrackerPriority.LOW]

        sim_time_mock = MagicMock()
        sim_time_mock.clock.secs = 1.0
        sim_time_mock.clock.nsecs = 9000.0
        tracker_manager._sim_time = sim_time_mock
        tracker_manager._callback_event.set()
        while tracker_manager._callback_event.is_set():
            time.sleep(0)
        tracker_manager.stop()
        assert tracker_manager._last_time == sim_time_mock.clock.secs + 1.e-9 * sim_time_mock.clock.nsecs
        tracker_mock.on_update_tracker.assert_not_called()
        tracker_mock2.on_update_tracker.assert_not_called()
        tracker_mock3.on_update_tracker.assert_not_called()

    def test_step_update_sim_time(self, rospy_mock):
        tracker_manager = TrackerManager(is_singleton=False)
        tracker_manager.start()
        assert not tracker_manager._callback_event.isSet()
        sim_time_mock = MagicMock()
        tracker_manager._update_sim_time(sim_time_mock)
        assert sim_time_mock == tracker_manager._sim_time
        assert tracker_manager._callback_event.isSet()

    def test_add(self, rospy_mock):
        tracker_manager = TrackerManager(is_singleton=False)
        tracker_mock = MagicMock()
        tracker_manager.add(tracker_mock)
        assert tracker_mock in tracker_manager._tracker_map[consts.TrackerPriority.NORMAL]

        tracker_mock2 = MagicMock()
        tracker_manager.add(tracker_mock2, consts.TrackerPriority.HIGH)
        assert tracker_mock2 in tracker_manager._tracker_map[consts.TrackerPriority.HIGH]

        tracker_mock3 = MagicMock()
        tracker_manager.add(tracker_mock3, consts.TrackerPriority.LOW)
        assert tracker_mock3 in tracker_manager._tracker_map[consts.TrackerPriority.LOW]

    def test_remove(self, rospy_mock):
        tracker_manager = TrackerManager(is_singleton=False)
        tracker_mock = MagicMock()
        tracker_manager.add(tracker_mock)
        assert tracker_mock in tracker_manager._tracker_map[consts.TrackerPriority.NORMAL]

        tracker_manager.remove(tracker_mock, consts.TrackerPriority.NORMAL)
        assert tracker_mock not in tracker_manager._tracker_map[consts.TrackerPriority.NORMAL]

    def test_remove_non_existing_tracker(self, rospy_mock):
        tracker_manager = TrackerManager(is_singleton=False)
        tracker_mock = MagicMock()
        with self.assertRaises(KeyError):
            tracker_manager.remove(tracker_mock, consts.TrackerPriority.NORMAL)

    def test_discard(self, rospy_mock):
        tracker_manager = TrackerManager(is_singleton=False)
        tracker_mock = MagicMock()

        tracker_manager.add(tracker_mock)
        assert tracker_mock in tracker_manager._tracker_map[consts.TrackerPriority.NORMAL]

        tracker_manager.discard(tracker_mock)
        assert tracker_mock not in tracker_manager._tracker_map[consts.TrackerPriority.NORMAL]

    def test_discard_non_existing_tracker(self, rospy_mock):
        tracker_manager = TrackerManager(is_singleton=False)
        tracker_mock = MagicMock()

        tracker_manager.discard(tracker_mock)
