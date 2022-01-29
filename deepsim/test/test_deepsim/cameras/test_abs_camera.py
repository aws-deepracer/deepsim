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
from typing import Any, Callable, Optional, Iterable
from unittest import TestCase
from unittest.mock import patch, MagicMock, call
import inspect

from deepsim.cameras.abs_camera import AbstractCamera
from deepsim.spawners.abs_model_spawner import AbstractModelSpawner
from deepsim.math.pose import Pose
from deepsim.exception import DeepSimCallbackError
from deepsim.constants import Tag

from rosgraph_msgs.msg import Clock

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class DummyCamera(AbstractCamera):
    def __init__(self, name: str,
                 spawner: Optional[AbstractModelSpawner] = None, *,
                 initial_pose: Optional[Pose] = None,
                 max_retry_attempts: int = 10,
                 backoff_time_sec: float = 1.0,
                 spawn_on_init: bool = True,
                 enable_on_init: bool = True,
                 tags: Optional[Iterable[str]] = None,
                 **kwargs) -> None:
        self.mock = MagicMock()
        super(DummyCamera, self).__init__(name=name,
                                          spawner=spawner,
                                          initial_pose=initial_pose,
                                          max_retry_attempts=max_retry_attempts,
                                          backoff_time_sec=backoff_time_sec,
                                          spawn_on_init=spawn_on_init,
                                          enable_on_init=enable_on_init,
                                          tags=tags,
                                          **kwargs)

    def spawn(self) -> None:
        self.mock.spawn()

    def on_reset_camera(self) -> None:
        self.mock.on_reset_camera()

    def on_update_camera(self, delta_time: float, sim_time: Clock) -> None:
        self.mock.on_update_camera(delta_time, sim_time)


@patch("deepsim.behaviours.behaviour_manager.BehaviourManager")
@patch("deepsim.behaviours.deepsim_behaviour.Transform")
class AbstractCameraTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, transform_mock, behaviour_manager_mock):
        name = myself()

        camera = DummyCamera(name)
        assert Tag.CAMERA in camera.tags

    def test_reset(self, transform_mock, behaviour_manager_mock):
        name = myself()

        camera = DummyCamera(name)
        camera.reset()

        camera.mock.on_reset_camera.assert_called_once()

    def test_reset_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        camera = DummyCamera(name)
        camera.mock.on_reset_camera.side_effect = Exception()

        with self.assertRaises(DeepSimCallbackError):
            camera.reset()

    def test_fixed_update(self, transform_mock, behaviour_manager_mock):
        name = myself()

        camera = DummyCamera(name)
        delta_time_mock = MagicMock()
        sim_time_mock = MagicMock()
        camera.fixed_update(delta_time=delta_time_mock,
                            sim_time=sim_time_mock)
        camera.mock.on_update_camera.assert_called_once_with(delta_time_mock,
                                                             sim_time_mock)

    def test_fixed_update_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        camera = DummyCamera(name)
        delta_time_mock = MagicMock()
        sim_time_mock = MagicMock()
        camera.mock.on_update_camera.side_effect = Exception()

        with self.assertRaises(DeepSimCallbackError):
            camera.fixed_update(delta_time=delta_time_mock,
                                sim_time=sim_time_mock)
