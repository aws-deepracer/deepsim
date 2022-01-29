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

from deepsim.spawners.dummy_spawner import DummySpawner

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class DummySpawnerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        spawner = DummySpawner()
        assert not spawner._should_validate_spawn
        assert not spawner._should_validate_delete

    def test_spawn(self):
        spawner = DummySpawner()
        with patch("deepsim.ros.ros_util.ROSUtil") as ros_util_mock:
            spawner.spawn(model_name=myself())
            ros_util_mock.wait_for_model_spawn.assert_not_called()

    def test_delete(self):
        spawner = DummySpawner()
        with patch("deepsim.ros.ros_util.ROSUtil") as ros_util_mock:
            spawner.delete(model_name=myself())
            ros_util_mock.wait_for_model_delete.assert_not_called()