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
"""A class for abstract camera."""
import abc
from typing import Optional, Iterable
import threading
from deepsim.behaviours.deepsim_behaviour import DeepSimBehaviour
from deepsim.spawners.abs_model_spawner import AbstractModelSpawner
from deepsim.core.pose import Pose
from deepsim.exception import DeepSimCallbackError
from deepsim.constants import Tag

from rosgraph_msgs.msg import Clock


class AbstractCamera(DeepSimBehaviour, metaclass=abc.ABCMeta):
    """
    Abstract Camera method
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
        Initialize Abstract Camera instance

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
        tags.add(Tag.CAMERA)
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
        self._lock = threading.RLock()

    def on_reset(self) -> None:
        """
        Reset the camera pose
        """
        with self._lock:
            try:
                self.on_reset_camera()
            except Exception:
                raise DeepSimCallbackError()

    def on_fixed_update(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update the camera pose

        Args:
            delta_time (float): time diff from last call
            sim_time (Clock): simulation time
        """
        with self._lock:
            try:
                self.on_update_camera(delta_time, sim_time)
            except Exception:
                raise DeepSimCallbackError()

    @abc.abstractmethod
    def on_reset_camera(self) -> None:
        """
        Reset the camera pose
        """
        raise NotImplementedError('Camera must implement on_reset_camera')

    @abc.abstractmethod
    def on_update_camera(self, delta_time: float, sim_time: Clock) -> None:
        """Update the camera pose

        Args:
            delta_time (float): time diff from last call
            sim_time (Clock): simulation time
        """
        raise NotImplementedError('Camera must implement on_update_camera')
