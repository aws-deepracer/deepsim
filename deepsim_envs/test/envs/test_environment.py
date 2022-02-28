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

from deepsim_envs.envs.environment import Environment

from deepsim import (
    Tag,
    GazeboServiceName,
    BehaviourManager
)

from std_srvs.srv import Empty, EmptyRequest


myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim_envs.envs.environment.ServiceProxyWrapper")
class EnvironmentTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, service_proxy_wrapper_mock):
        agent_mock = MagicMock()
        area_mock = MagicMock()
        area_mock.get_agents.return_value = [agent_mock]
        env = Environment(area=area_mock)
        service_proxy_wrapper_mock.has_call(
            call(GazeboServiceName.PAUSE_PHYSICS, Empty),
            call(GazeboServiceName.UNPAUSE_PHYSICS, Empty),
        )
        assert area_mock == env._area
        area_mock.reset.assert_called_once()

    def test_reset(self, service_proxy_wrapper_mock):
        agent_mock = MagicMock()
        agent_mock.name = myself()
        agent_mock.tags = {Tag.AGENT}
        next_state_mock = MagicMock()
        info_mock = MagicMock()
        agent_mock.get_next_state.return_value = next_state_mock
        BehaviourManager.get_instance().add(agent_mock)

        pause_physics_mock = MagicMock()
        unpause_physics_mock = MagicMock()

        def service_creator(service_name, service_class):
            if service_name == GazeboServiceName.PAUSE_PHYSICS:
                return pause_physics_mock
            if service_name == GazeboServiceName.UNPAUSE_PHYSICS:
                return unpause_physics_mock
            raise Exception("Unknown service name given: {}".format(service_name))
        service_proxy_wrapper_mock.side_effect = service_creator

        area_mock = MagicMock()
        area_mock.get_agents.return_value = [agent_mock]
        area_mock.get_info.return_value = info_mock
        env = Environment(area=area_mock)
        next_state, info = env.reset()
        assert next_state[myself()] == next_state_mock
        assert info == info_mock
        assert area_mock.reset.call_count == 2
        assert agent_mock.reset.call_count == 2
        assert agent_mock.get_next_state.call_count == 2
        assert agent_mock.on_episode_begin.call_count == 2
        assert pause_physics_mock.call_count == 4
        assert unpause_physics_mock.call_count == 4

    def test_step(self, service_proxy_wrapper_mock):
        agent_mock = MagicMock()
        agent_mock.name = myself()
        agent_mock.tags = {Tag.AGENT}
        next_state_mock = MagicMock()
        agent_mock.get_next_state.return_value = next_state_mock
        agent_mock.last_step_reward = 10.0
        agent_mock.done = False
        BehaviourManager.get_instance().add(agent_mock)

        pause_physics_mock = MagicMock()
        unpause_physics_mock = MagicMock()

        def service_creator(service_name, service_class):
            if service_name == GazeboServiceName.PAUSE_PHYSICS:
                return pause_physics_mock
            if service_name == GazeboServiceName.UNPAUSE_PHYSICS:
                return unpause_physics_mock
            raise Exception("Unknown service name given: {}".format(service_name))

        service_proxy_wrapper_mock.side_effect = service_creator

        area_mock = MagicMock()
        area_mock.get_agents.return_value = [agent_mock]
        env = Environment(area=area_mock)
        action_dict = {myself(): 3}
        next_state, reward_dict, done_dict, action_dict, info = env.step(action_dict)
        agent_mock.act.assert_called_once_with(3)
        agent_mock.update.assert_called_once()
        assert agent_mock.get_next_state.call_count == 2
        assert pause_physics_mock.call_count == 3
        assert unpause_physics_mock.call_count == 3

        assert next_state[myself()] == next_state_mock
        assert reward_dict[myself()] == 10.0
        assert not done_dict[myself()]
        assert action_dict[myself()] == 3

    def test_close(self, service_proxy_wrapper_mock):
        agent_mock = MagicMock()
        area_mock = MagicMock()
        area_mock.get_agents.return_value = [agent_mock]
        env = Environment(area=area_mock)
        env.close()
        area_mock.close.assert_called_once()

    def test_observation_space(self, service_proxy_wrapper_mock):
        agent_mock = MagicMock()
        area_mock = MagicMock()
        area_mock.get_agents.return_value = [agent_mock]
        env = Environment(area=area_mock)
        observation_space = env.observation_space
        assert observation_space == area_mock.observation_space

    def test_action_space(self, service_proxy_wrapper_mock):
        agent_mock = MagicMock()
        area_mock = MagicMock()
        area_mock.get_agents.return_value = [agent_mock]
        env = Environment(area=area_mock)
        action_space = env.action_space
        assert action_space == area_mock.action_space
