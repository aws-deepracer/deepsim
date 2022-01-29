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

from deepsim.sim_trackers.tracker import TrackerInterface
import deepsim.sim_trackers.constants as consts

from rosgraph_msgs.msg import Clock


class DummyTracker(TrackerInterface):
    def __init__(self, priority: consts.TrackerPriority = consts.TrackerPriority.NORMAL):
        self.mock = MagicMock()

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        self.mock.on_update_tracker(delta_time, sim_time)


@patch("deepsim.sim_trackers.tracker_manager.TrackerManager")
class AbstractTrackerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_on_update_tracker(self, tracker_manager_mock):
        tracker = DummyTracker()
        delta_time_mock = MagicMock()
        sim_time_mock = MagicMock()
        tracker.on_update_tracker(delta_time=delta_time_mock,
                                  sim_time=sim_time_mock)
        tracker.mock.on_update_tracker.assert_called_once_with(delta_time_mock,
                                                               sim_time_mock)
