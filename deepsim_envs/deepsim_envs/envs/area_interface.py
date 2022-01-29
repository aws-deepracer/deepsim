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
"""A class for abstract area."""
import abc
from typing import Iterable, Dict
from deepsim_envs.agents.abs_agent import AbstractAgent

from ude import AgentID, Space

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class AreaInterface(ABC):
    """
    Area interface class.
    """
    @abc.abstractmethod
    def get_agents(self) -> Iterable[AbstractAgent]:
        """
        Returns agents in the area.

        Returns:
            Iterable[AbstractAgent]: the agents in the area.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def get_info(self) -> dict:
        """
        Returns area information.

        Returns:
            dict: the area information.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def reset(self) -> None:
        """
        Reset the area
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def close(self) -> None:
        """
        Close the area
        """
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def observation_space(self) -> Dict[AgentID, Space]:
        """
        Returns the observation spaces of agents in env.

        Returns:
            Dict[AgentID, Space]: the observation spaces of agents in env.
        """
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def action_space(self) -> Dict[AgentID, Space]:
        """
        Returns the action spaces of agents in env.

        Returns:
            Dict[AgentID, Space]: the action spaces of agents in env.
        """
        raise NotImplementedError()
