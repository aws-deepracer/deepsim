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

from deepsim.behaviours.deepsim_behaviour import DeepSimBehaviour
from deepsim.spawners.abs_model_spawner import AbstractModelSpawner
from deepsim.math.pose import Pose
from deepsim.math.vector3 import Vector3
from deepsim.math.quaternion import Quaternion
from deepsim.spawners.dummy_spawner import DummySpawner
from deepsim.exception import DeepSimCallbackError

from rosgraph_msgs.msg import Clock

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class DummyBehaviour(DeepSimBehaviour):
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
        super(DummyBehaviour, self).__init__(name=name,
                                             spawner=spawner,
                                             initial_pose=initial_pose,
                                             max_retry_attempts=max_retry_attempts,
                                             backoff_time_sec=backoff_time_sec,
                                             spawn_on_init=spawn_on_init,
                                             enable_on_init=enable_on_init,
                                             tags=tags,
                                             **kwargs)

    def on_spawn(self) -> None:
        self.mock.on_spawn()

    def on_enable(self) -> None:
        self.mock.on_enable()

    def on_disable(self) -> None:
        self.mock.on_disable()

    def on_delete(self) -> None:
        self.mock.on_delete()

    def on_update(self) -> None:
        self.mock.on_update()

    def on_reset(self) -> None:
        self.mock.on_reset()

    def on_fixed_update(self, delta_time: float, sim_time: Clock) -> None:
        self.mock.on_fixed_update(delta_time, sim_time)


@patch("deepsim.behaviours.behaviour_manager.BehaviourManager")
@patch("deepsim.behaviours.deepsim_behaviour.Transform")
class DeepSimBehaviourTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, transform_mock, behaviour_manager_mock):
        name = myself()
        dummy_spawner = DummySpawner()
        behaviour = DummyBehaviour(name, spawner=dummy_spawner)
        assert behaviour.name == name
        assert behaviour._spawner == dummy_spawner
        assert behaviour.initial_pose == Pose()
        assert behaviour.max_retry_attempts == 10
        assert behaviour.backoff_time_sec == 1.0

        behaviour.mock.on_spawn.assert_called_once()
        transform_mock.assert_called_once_with(behaviour=behaviour,
                                               name=name)
        behaviour_manager_mock.get_instance.return_value.add.assert_called_once_with(behaviour)

        assert behaviour.is_enabled

        new_pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                        orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        behaviour.initial_pose = new_pose
        assert behaviour.initial_pose == new_pose

        behaviour.max_retry_attempts = 3
        assert behaviour.max_retry_attempts == 3

        behaviour.backoff_time_sec = 3.0
        assert behaviour.backoff_time_sec == 3.0

    def test_initialize_no_spawn_on_init(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name,
                                   spawn_on_init=False)
        assert name == behaviour.name
        behaviour.mock.on_spawn.assert_not_called()
        transform_mock.assert_called_once_with(behaviour=behaviour,
                                               name=name)
        behaviour_manager_mock.get_instance.return_value.add.assert_called_once_with(behaviour)

        assert behaviour.is_enabled

    def test_initialize_disable_on_init(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name,
                                   enable_on_init=False)

        behaviour.mock.on_spawn.assert_called_once()
        transform_mock.assert_called_once_with(behaviour=behaviour,
                                               name=name)
        behaviour_manager_mock.get_instance.return_value.add.assert_called_once_with(behaviour)

        assert not behaviour.is_enabled

    def test_initiaze_tags(self, transform_mock, behaviour_manager_mock):
        name = myself()
        behaviour = DummyBehaviour(name,
                                   tags=["test_tag"])

        assert "test_tag" in behaviour.tags

    def test_spawner_args(self, transform_mock, behaviour_manager_mock):
        name = myself()
        with patch("deepsim.behaviours.deepsim_behaviour.DummySpawner") as dummy_spawner_mock:
            behaviour = DummyBehaviour(name,
                                       test="test_arg")
            dummy_spawner_mock.return_value.spawn.assert_called_once_with(model_name=name,
                                                                          initial_pose=Pose(),
                                                                          max_retry_attempts=10,
                                                                          backoff_time_sec=1.0,
                                                                          test="test_arg")
            behaviour.mock.on_spawn.assert_called_once()

    def test_set_spawner_args(self, transform_mock, behaviour_manager_mock):
        name = myself()
        with patch("deepsim.behaviours.deepsim_behaviour.DummySpawner") as dummy_spawner_mock:
            behaviour = DummyBehaviour(name,
                                       spawner=dummy_spawner_mock(),
                                       spawn_on_init=False)
            new_pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                            orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
            behaviour.initial_pose = new_pose
            behaviour.set_spawner_args(test="test_arg")
            behaviour.spawn()
            dummy_spawner_mock.return_value.spawn.assert_called_once_with(model_name=name,
                                                                          initial_pose=new_pose,
                                                                          max_retry_attempts=10,
                                                                          backoff_time_sec=1.0,
                                                                          test="test_arg")
            behaviour.mock.on_spawn.assert_called_once()

    def test_is_enable_without_transform(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name,
                                   enable_on_init=False)
        behaviour._transform = None
        assert not behaviour.is_enabled

    def test_enable(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name,
                                   enable_on_init=False)

        assert not behaviour.is_enabled
        behaviour.enable()
        assert behaviour.is_enabled
        behaviour.mock.on_enable.assert_called_once()

    def test_enable_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name,
                                   enable_on_init=False)

        behaviour.mock.on_enable.side_effect = Exception()
        with self.assertRaises(DeepSimCallbackError):
            behaviour.enable()
            behaviour.mock.on_enable.assert_not_called()

    def test_enable_without_transform(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name,
                                   enable_on_init=False)

        behaviour._transform = None
        with self.assertRaises(RuntimeError):
            behaviour.enable()
            behaviour.mock.on_enable.assert_not_called()

    def test_disable(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)

        assert behaviour.is_enabled
        behaviour.disable()
        assert not behaviour.is_enabled
        behaviour.mock.on_disable.assert_called_once()

    def test_disable_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)

        behaviour.mock.on_disable.side_effect = Exception()
        with self.assertRaises(DeepSimCallbackError):
            behaviour.disable()
            behaviour.mock.on_disable.assert_not_called()

    def test_disable_without_transform(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)

        behaviour._transform = None
        behaviour.disable()
        behaviour.mock.on_disable.assert_not_called()

    def test_update(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)
        behaviour.update()
        behaviour.mock.on_update.assert_called_once()

    def test_update_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)
        behaviour.mock.on_update.side_effect = Exception()

        with self.assertRaises(DeepSimCallbackError):
            behaviour.update()

    def test_ignore_update_when_disabled(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)
        behaviour.disable()
        behaviour.update()
        behaviour.mock.on_update.assert_not_called()

    def test_reset(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)
        behaviour.reset()
        behaviour.mock.on_reset.assert_called_once()

    def test_reset_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)
        behaviour.mock.on_reset.side_effect = Exception()

        with self.assertRaises(DeepSimCallbackError):
            behaviour.reset()

    def test_ignore_reset_when_disabled(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)
        behaviour.disable()
        behaviour.reset()
        behaviour.mock.on_reset.assert_not_called()

    def test_fixed_update(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)
        delta_time_mock = MagicMock()
        sim_time_mock = MagicMock()
        behaviour.fixed_update(delta_time=delta_time_mock,
                               sim_time=sim_time_mock)
        behaviour.mock.on_fixed_update.assert_called_once_with(delta_time_mock,
                                                               sim_time_mock)

    def test_fixed_update_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)
        delta_time_mock = MagicMock()
        sim_time_mock = MagicMock()
        behaviour.mock.on_fixed_update.side_effect = Exception()

        with self.assertRaises(DeepSimCallbackError):
            behaviour.fixed_update(delta_time=delta_time_mock,
                                   sim_time=sim_time_mock)

    def test_ignore_fixed_update_when_disabled(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)
        delta_time_mock = MagicMock()
        sim_time_mock = MagicMock()

        behaviour.disable()
        behaviour.fixed_update(delta_time=delta_time_mock,
                               sim_time=sim_time_mock)
        behaviour.mock.on_fixed_update.assert_not_called()

    def test_delete(self, transform_mock, behaviour_manager_mock):
        name = myself()

        behaviour = DummyBehaviour(name)

        behaviour.delete()
        behaviour_manager_mock.get_instance.return_value.discard.assert_called_once_with(behaviour)
        transform_mock.return_value.delete.assert_called_once_with(transform_mock.return_value.name)
        assert behaviour.transform is None
        assert not behaviour.is_enabled
        behaviour.mock.on_delete.assert_called_once()

    def test_delete_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()
        with patch("deepsim.behaviours.deepsim_behaviour.DummySpawner") as dummy_spawner_mock:
            behaviour = DummyBehaviour(name,
                                       test="test_val")
            behaviour.mock.on_delete.side_effect = Exception()
            with self.assertRaises(Exception):
                behaviour.delete()

            behaviour_manager_mock.get_instance.return_value.discard.assert_called_once_with(behaviour)
            transform_mock.return_value.delete.assert_called_once_with(transform_mock.return_value.name)
            assert behaviour.transform is None
            assert not behaviour.is_enabled
            dummy_spawner_mock.return_value.delete.assert_called_once_with(model_name=name,
                                                                           max_retry_attempts=10,
                                                                           backoff_time_sec=1.0,
                                                                           test="test_val")
