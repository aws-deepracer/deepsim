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

from deepsim_envs.agents.abs_agent import AbstractAgent

from deepsim import (
    DeepSimCallbackError,
    Tag,
    AbstractModelSpawner,
    Pose
)

from rosgraph_msgs.msg import Clock

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class DummyAgent(AbstractAgent):
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
        super(DummyAgent, self).__init__(name=name,
                                         spawner=spawner,
                                         initial_pose=initial_pose,
                                         max_retry_attempts=max_retry_attempts,
                                         backoff_time_sec=backoff_time_sec,
                                         spawn_on_init=spawn_on_init,
                                         enable_on_init=enable_on_init,
                                         tags=tags,
                                         **kwargs)

    def on_enable(self) -> None:
        self.mock.on_enable()

    def on_disable(self) -> None:
        self.mock.on_disable()

    def on_delete(self) -> None:
        self.mock.on_delete()

    def on_update(self) -> None:
        self.mock.on_update()

    def on_fixed_update(self, delta_time: float, sim_time: Clock) -> None:
        self.mock.on_fixed_update(delta_time, sim_time)

    def on_reset_agent(self) -> None:
        self.mock.on_reset_agent()

    def on_action_received(self, action: Any) -> None:
        self.mock.on_action_received(action)

    def get_next_state(self) -> Any:
        return self.mock.get_next_state()

    def on_episode_begin(self) -> None:
        self.mock.on_episode_begin()

    def on_episode_end(self) -> None:
        self.mock.on_episode_end()


@patch("deepsim.behaviours.behaviour_manager.BehaviourManager")
@patch("deepsim.behaviours.deepsim_behaviour.Transform")
class AbstractAgentTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, transform_mock, behaviour_manager_mock):
        name = myself()

        agent = DummyAgent(name)
        assert Tag.AGENT in agent.tags
        assert not agent.done
        assert agent.last_step_reward == 0.0
        assert agent.episode_reward == 0.0
        assert agent.last_action is None

    def test_reward_setter(self, transform_mock, behaviour_manager_mock):
        name = myself()

        agent = DummyAgent(name)
        assert agent.last_step_reward == 0.0
        assert agent.episode_reward == 0.0

        agent.add_reward(5.0)
        assert agent.last_step_reward == 5.0
        assert agent.episode_reward == 5.0

        # On next step
        agent.add_reward(3.0)
        assert agent.last_step_reward == 3.0
        assert agent.episode_reward == 8.0

    def test_act(self, transform_mock, behaviour_manager_mock):
        name = myself()

        agent = DummyAgent(name)

        action = MagicMock()

        agent.act(action)

        assert agent.last_action == action
        agent.mock.on_action_received.assert_called_once_with(action)

    def test_act_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        agent = DummyAgent(name)
        agent.mock.on_action_received.side_effect = Exception()

        action = MagicMock()
        with self.assertRaises(DeepSimCallbackError):
            agent.act(action)

        assert agent.last_action == action

    def test_end_episode(self, transform_mock, behaviour_manager_mock):
        name = myself()

        agent = DummyAgent(name)

        assert not agent.done
        agent.end_episode()
        assert agent.done

        agent.mock.on_episode_end.assert_called_once()

    def test_end_episode_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        agent = DummyAgent(name)
        agent.mock.on_episode_end.side_effect = Exception()

        with self.assertRaises(DeepSimCallbackError):
            assert not agent.done
            agent.end_episode()

        assert agent.done

    def test_reset(self, transform_mock, behaviour_manager_mock):
        name = myself()

        agent = DummyAgent(name)

        agent.end_episode()
        agent.add_reward(3.0)
        agent.add_reward(5.0)
        assert agent.done
        assert agent.last_step_reward == 5.0
        assert agent.episode_reward == 8.0
        agent.reset()
        assert agent.last_step_reward == 0.0
        assert agent.episode_reward == 0.0
        assert not agent.done

        agent.mock.on_reset_agent.assert_called_once()

    def test_reset_callback_error(self, transform_mock, behaviour_manager_mock):
        name = myself()

        agent = DummyAgent(name)
        agent.mock.on_reset_agent.side_effect = Exception()

        with self.assertRaises(DeepSimCallbackError):
            agent.end_episode()
            agent.add_reward(3.0)
            agent.add_reward(5.0)
            assert agent.done
            assert agent.last_step_reward == 5.0
            assert agent.episode_reward == 8.0
            agent.reset()
        assert not agent.done
        assert agent.last_step_reward == 0.0
        assert agent.episode_reward == 0.0
