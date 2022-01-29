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
"""A class for environment."""
import abc
from threading import RLock
from typing import Dict

from deepsim_envs.envs.area_interface import AreaInterface

from deepsim import (
    BehaviourManager,
    ServiceProxyWrapper,
    GazeboServiceName,
    DeepSimException
)

from ude import (
    MultiAgentDict,
    UDEStepResult,
    AgentID,
    Space
)
from ude_ros_env.ros_env_interface import ROSEnvironmentInterface


from std_srvs.srv import Empty, EmptyRequest


# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class Environment(ROSEnvironmentInterface):
    """
    Environment class
    """
    def __init__(self, area: AreaInterface) -> None:
        """
        Initialize Environment

        Args:
            area (AreaInterface): area of the environment.
        """
        self._lock = RLock()
        self._area = area
        self.pause_physics = ServiceProxyWrapper(GazeboServiceName.PAUSE_PHYSICS, Empty)
        self.unpause_physics = ServiceProxyWrapper(GazeboServiceName.UNPAUSE_PHYSICS, Empty)
        self.reset()

    def reset(self) -> MultiAgentDict:
        """
        Reset the environment

        Returns:
            MultiAgentDict: new observations for each of the agents in the environment.
        """
        with self._lock:
            # Reset the area.
            # During resetting area, there might be spawning/deletion of models.
            # Thus, we need to unpause physics prior to area reset.
            self.unpause_physics(EmptyRequest())
            self._area.reset()
            self.pause_physics(EmptyRequest())

            agents = list(self._area.get_agents())
            if len(agents) <= 0:
                raise DeepSimException("Number of agents must be > 0!")

            # Reset all behaviours in the area.
            BehaviourManager.get_instance().reset()

            # Get next observation of all agents in the area.
            # Requires to unpause physics in order to get next sensor data.
            self.unpause_physics(EmptyRequest())
            next_state_dict = {agent.name: agent.get_next_state() for agent in agents}
            self.pause_physics(EmptyRequest())

            [agent.on_episode_begin() for agent in agents]
            return next_state_dict

    def step(self, action_dict: MultiAgentDict) -> UDEStepResult:
        """
        Take a step with given agents' actions.

        Args:
            action_dict (MultiAgentDict): the actions for each of the agents in the environment.

        Returns:
            StepResult: observations, rewards, dones, last_actions, info
        """
        with self._lock:
            agents = list(self._area.get_agents())
            # If there is no action given for agent then assign action as None
            action_dict = {agent.name: action_dict[agent.name] if agent.name in action_dict else None
                           for agent in agents}
            [agent.act(action_dict[agent.name]) for agent in agents]
            self.unpause_physics(EmptyRequest())
            next_state_dict = {agent.name: agent.get_next_state() for agent in agents}
            self.pause_physics(EmptyRequest())
            BehaviourManager.get_instance().update()

            reward_dict = {agent.name: agent.last_step_reward for agent in agents}
            done_dict = {agent.name: agent.done for agent in agents}

            info = self._area.get_info() or {}
            return next_state_dict, reward_dict, done_dict, action_dict, info

    def close(self) -> None:
        """
        Close environment.
        """
        with self._lock:
            self._area.close()

    @property
    def observation_space(self) -> Dict[AgentID, Space]:
        """
        Returns the observation spaces of agents in env.

        Returns:
            Dict[AgentID, Space]: the observation spaces of agents in env.
        """
        with self._lock:
            return self._area.observation_space

    @property
    def action_space(self) -> Dict[AgentID, Space]:
        """
        Returns the action spaces of agents in env.

        Returns:
            Dict[AgentID, Space]: the action spaces of agents in env.
        """
        with self._lock:
            return self._area.action_space
