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
"""A class for DeepSim behaviour."""
import abc
import copy
from typing import Optional, Iterable
from threading import RLock

from deepsim.behaviours.transform import Transform
from deepsim.math.pose import Pose
from deepsim.spawners.abs_model_spawner import AbstractModelSpawner
from deepsim.spawners.dummy_spawner import DummySpawner
from deepsim.exception import DeepSimCallbackError

from rosgraph_msgs.msg import Clock

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class DeepSimBehaviour(ABC):
    """
    Abstract Behaviour class
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
        Initialize Abstract Behaviour class

        Args:
            name (str): the name of the model.
            initial_pose (Optional[Pose]): initial model spawn pose
            max_retry_attempts (int): max retry attempts for waiting spawn/delete to complete
            backoff_time_sec (float): backoff time in seconds for spawn/delete to complete
            spawn_on_init (bool): the flag whether to spawn the model on initialization.
            enable_on_init (bool): the flag whether to enable the model on initialization.
            tags: (Optional[Iterable[str]]): The tags in iterable type.
            **kwargs: Arbitrary keyword arguments for spawner
        """
        from deepsim.behaviours.behaviour_manager import BehaviourManager

        self._transform_lock = RLock()

        self._name = name
        self._tags = set()
        if tags:
            self._tags.update(tags)

        self._is_enabled = False

        self._is_spawned = False
        self._spawner = spawner or DummySpawner()
        self._spawner_args = kwargs
        self._initial_pose = initial_pose or Pose()
        self._max_retry_attempts = max_retry_attempts
        self._backoff_time_sec = backoff_time_sec

        if spawn_on_init:
            self.spawn()

        # Only add transform and behaviour with successful spawn if spawning on init.
        self._transform = Transform(behaviour=self,
                                    name=name)
        BehaviourManager.get_instance().add(self)

        if enable_on_init:
            self.enable()

    @property
    def name(self) -> str:
        """
        Returns the name of behaviour.

        Returns:
            str: the name of behaviour.
        """
        return self._name

    @property
    def initial_pose(self) -> Pose:
        """
        Return a copy of initial pose.

        Returns:
            Pose: a copy of initial pose.
        """
        return self._initial_pose.copy()

    @initial_pose.setter
    def initial_pose(self, value: Pose) -> None:
        """
        Set new initial pose.

        Args:
            value (Pose): new initial pose.
        """
        self._initial_pose = value.copy()

    @property
    def max_retry_attempts(self) -> int:
        """
        Returns max retry attempts for spawn or delete.

        Returns:
            int: max retry attempts for spawn or delete.
        """
        return self._max_retry_attempts

    @max_retry_attempts.setter
    def max_retry_attempts(self, value: int) -> None:
        """
        Sets max retry attempts number.

        Args:
            value (int): new max retry attempts number.
        """
        self._max_retry_attempts = value

    @property
    def backoff_time_sec(self) -> float:
        """
        Returns backoff time in sec for spawn and delete.

        Returns:
            float: backoff time in sec for spawn and delete.
        """
        return self._backoff_time_sec

    @backoff_time_sec.setter
    def backoff_time_sec(self, value: float) -> None:
        """
        Sets new backoff time in sec for spawn and delete.

        Args:
            value (float): new backoff time in sec for spawn and delete.
        """
        self._backoff_time_sec = value

    @property
    def tags(self) -> set:
        """
        Returns the tags of behaviour.

        Returns:
            set: the tags of behaviour.
        """
        return copy.copy(self._tags)

    @property
    def transform(self) -> Transform:
        """
        Returns the transform object.

        Returns:
            Transform: the transform object of this behaviour.
        """
        # Behaviour can use transform to access all Gazebo model information
        # and manipulate the Gazebo model (pose/velocity/angular_velocity/visuals)
        return self._transform

    @property
    def is_enabled(self) -> bool:
        """
        Returns whether this behaviour is enabled or not.

        Returns:
            bool: the flag whether this behaviour is enabled or not.
        """
        return self._is_enabled if self.transform else False

    def set_spawner_args(self, **kwargs) -> None:
        """
        Sets new spawner args.

        Args:
            **kwargs: arbitrary keyword arguments for spawner.
        """
        self._spawner_args = kwargs

    def spawn(self) -> None:
        """
        Spawn the Gazebo mode.
        * If the model is already spawned then it will delete and spawn again.
        """
        try:
            self.on_spawn()
        except Exception:
            raise DeepSimCallbackError()
        finally:
            if self._is_spawned:
                self._spawner.delete(model_name=self.name,
                                     max_retry_attempts=self.max_retry_attempts,
                                     backoff_time_sec=self.backoff_time_sec,
                                     **self._spawner_args)
            self._is_spawned = self._spawner.spawn(model_name=self.name,
                                                   initial_pose=self._initial_pose,
                                                   max_retry_attempts=self.max_retry_attempts,
                                                   backoff_time_sec=self.backoff_time_sec,
                                                   **self._spawner_args)

    def enable(self) -> None:
        """
        Enable the behaviour
        """
        # Enable this behaviour
        with self._transform_lock:
            if self._transform:
                try:
                    self.on_enable()
                except Exception:
                    raise DeepSimCallbackError()
                finally:
                    self._is_enabled = True
            else:
                raise RuntimeError("Cannot enable deleted object")

    def disable(self) -> None:
        """
        Disable the behaviour.
        """
        # Disable this behaviour
        with self._transform_lock:
            if self._transform:
                try:
                    self.on_disable()
                except Exception:
                    raise DeepSimCallbackError()
                finally:
                    self._is_enabled = False

    def update(self) -> None:
        """
        Update the behaviour.
        """
        if self.is_enabled:
            try:
                self.on_update()
            except Exception:
                raise DeepSimCallbackError()

    def reset(self) -> None:
        """
        Reset the behaviour.
        """
        if self.is_enabled:
            try:
                self.on_reset()
            except Exception:
                raise DeepSimCallbackError()

    def fixed_update(self, delta_time: float, sim_time: Clock) -> None:
        """
        Fixed update from simulator tick.

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        if self.is_enabled:
            try:
                self.on_fixed_update(delta_time, sim_time)
            except Exception:
                raise DeepSimCallbackError()

    def delete(self) -> None:
        """
        Delete the behaviour.
        * Note: Once deleted, the object cannot be re-used.
        """
        # Delete this behaviour
        with self._transform_lock:
            if self._transform:
                try:
                    self.on_delete()
                except Exception:
                    raise DeepSimCallbackError()
                finally:
                    from deepsim.behaviours.behaviour_manager import BehaviourManager
                    BehaviourManager.get_instance().discard(self)
                    self.transform.delete(self.transform.name)
                    if self._is_spawned:
                        self._spawner.delete(model_name=self.name,
                                             max_retry_attempts=self.max_retry_attempts,
                                             backoff_time_sec=self.backoff_time_sec,
                                             **self._spawner_args)
                    self._transform = None
                    self._is_enabled = False

    def on_enable(self) -> None:
        """
        Callback on enable (called right before behaviour is enabled)
        """
        pass

    def on_disable(self) -> None:
        """
        Callback on disable (called right before behaviour is disabled)
        """
        pass

    def on_spawn(self) -> None:
        """
        Callback on spawn (called right before behaviour is spawned)
        """
        pass

    def on_delete(self) -> None:
        """
        Callback on delete (called right before behaviour is deleted)
        """
        pass

    def on_update(self) -> None:
        """
        Update callback (update from Environment)
        """
        pass

    def on_reset(self) -> None:
        """
        Reset callback (reset from Environment)
        """
        pass

    def on_fixed_update(self, delta_time: float, sim_time: Clock) -> None:
        """
        Gazebo Simulator Clock callback (update from Gazebo tick)
        This should be used extremely carefully...
        In most of cases, you don't need to implement this and
        any update can be done with on_update callback.
        This should be used only when you need to update things every Gazebo tick.

        Since this will be called every tick of Gazebo simulator and
        the same thread is used for setting and getting the Gazebo model information,
        the callback execution must be very lightweight and short.
        if this takes long time to execute, this may impact overall simulation.

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        pass
