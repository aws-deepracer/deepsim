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
"""A class for abstract agent."""
import abc
from typing import Optional, Any, Iterable

from deepsim import (
    DeepSimBehaviour,
    DeepSimCallbackError,
    Tag,
    AbstractModelSpawner,
    Pose
)

from threading import RLock


class AbstractAgent(DeepSimBehaviour, metaclass=abc.ABCMeta):
    """
    Abstract Agent Class
    """
    def __init__(self, name: str,
                 spawner: Optional[AbstractModelSpawner] = None, *,
                 initial_pose: Optional[Pose] = None,
                 max_retry_attempts: int = 10,
                 backoff_time_sec: float = 1.0,
                 spawn_on_init: bool = True,
                 enable_on_init: bool = True,
                 tags: Optional[Iterable[str]] = None,
                 **kwargs) -> None:
        """
        Initialize Abstract Agent class

        Args:
            name (str): the name of the model.
            initial_pose (Optional[Pose]): initial model spawn pose
            max_retry_attempts (int): max retry attempts for waiting spawn/delete to complete
            backoff_time_sec (float): backoff time in seconds for spawn/delete to complete
            spawn_on_init (bool): the flag whether to spawn the model on initialization.
            enable_on_init (bool): the flag whether to enable the model on initialization.
            tags: (Optional[Iterable[str]]): The tags in iterable type.
            kwargs: the keyword arguments for spawn method.
        """
        tags = set(tags) if tags else set()
        tags.add(Tag.AGENT)
        DeepSimBehaviour.__init__(self,
                                  name=name,
                                  spawner=spawner,
                                  initial_pose=initial_pose,
                                  max_retry_attempts=max_retry_attempts,
                                  backoff_time_sec=backoff_time_sec,
                                  spawn_on_init=spawn_on_init,
                                  enable_on_init=enable_on_init,
                                  tags=tags,
                                  **kwargs)
        self._sync_lock = RLock()
        self._done = False
        self._last_step_reward = 0.0
        self._episode_reward = 0.0
        self._last_action = None

    @property
    def last_action(self) -> Any:
        """
        Returns the last action taken.

        Returns:
            Any: the last action taken.
        """
        return self._last_action

    @property
    def done(self) -> bool:
        """
        Returns the flag whether this agent finished the episode.

        Returns:
            bool: the flag whether the agent finished the episode or not.
        """
        return self._done

    @property
    def episode_reward(self) -> float:
        """
        Returns the episode reward.

        Returns:
            float: the episode reward.
        """
        return self._episode_reward

    @property
    def last_step_reward(self) -> float:
        """
        Returns the state reward.

        Returns:
            float: the state reward.
        """
        return self._last_step_reward

    def add_reward(self, reward: float) -> None:
        """
        Set state reward.

        Args:
            reward (float): the state reward
        """
        with self._sync_lock:
            self._last_step_reward = reward
            self._episode_reward += reward

    def act(self, action: Any) -> None:
        """
        Take the given action on current step.

        Args:
            action (Any): the action to take on current step
        """
        with self._sync_lock:
            try:
                self.on_action_received(action)
            except Exception:
                raise DeepSimCallbackError()
            finally:
                self._last_action = action

    def end_episode(self) -> None:
        """
        End the current episode.
        """
        with self._sync_lock:
            try:
                self.on_episode_end()
            except Exception:
                raise DeepSimCallbackError()
            finally:
                self._done = True

    def on_reset(self) -> None:
        """
        Reset the agent to start new episode.
        """
        with self._sync_lock:
            try:
                self.on_reset_agent()
            except Exception:
                raise DeepSimCallbackError()
            finally:
                self._done = False
                self._last_step_reward = 0.0
                self._episode_reward = 0.0

    def on_reset_agent(self) -> None:
        """
        Callback on reset (called right before reset).
        """
        pass

    @abc.abstractmethod
    def on_action_received(self, action: Any) -> None:
        """
        Callback on action received

        Args:
            action (Any): the action received.
        """
        raise NotImplementedError('Agent must implement on_action_received method!')

    @abc.abstractmethod
    def get_next_state(self) -> Any:
        """
        Subclass must implement this to retrieve next state and return.

        Returns:
            Any: the next state observation.
        """
        raise NotImplementedError('Agent must implement get_next_state method!')

    def on_episode_begin(self) -> None:
        """
        Callback on episode begin (called right before new episode starts)
        """
        pass

    def on_episode_end(self) -> None:
        """
        Callback on episode end (called right before current episode end for this agent)
        """
        pass
