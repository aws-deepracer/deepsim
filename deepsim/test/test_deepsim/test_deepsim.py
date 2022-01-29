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

from deepsim.deepsim import DeepSim


class DeepSimTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_timestep(self):
        deepsim = DeepSim(is_singleton=False)
        deepsim.timestep = 3.0
        assert deepsim.timestep == 3.0

    def test_start(self):
        with patch("deepsim.sim_trackers.tracker_manager.TrackerManager") as tracker_manager_mock:
            deepsim = DeepSim(is_singleton=False)
            deepsim.start()
            tracker_manager_mock.get_instance.return_value.start.assert_called_once()

    def test_stop(self):
        with patch("deepsim.sim_trackers.tracker_manager.TrackerManager") as tracker_manager_mock:
            deepsim = DeepSim(is_singleton=False)
            deepsim.stop()
            tracker_manager_mock.get_instance.return_value.stop.assert_called_once()
