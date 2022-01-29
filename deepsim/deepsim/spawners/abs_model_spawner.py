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
"""A class for abstract model spawner."""
import abc
from threading import RLock
from typing import Optional

from deepsim.math.pose import Pose
from deepsim.ros.ros_util import ROSUtil

import rospy


class AbstractModelSpawner(object, metaclass=abc.ABCMeta):
    """
    Abstract model spawner class

    Attributes
        _model_names (set): set of model names
    """
    _model_names = set()
    _model_name_lock = RLock()

    def __init__(self,
                 should_validate_spawn: bool = True,
                 should_validate_delete: bool = True):
        """
        Constructor

        Args:
            should_validate_spawn (bool): flag whether to validate spawn
            should_validate_delete (bool): flag whether to validate delete
        """
        self._should_validate_spawn = should_validate_spawn
        self._should_validate_delete = should_validate_delete

    def spawn(self,
              model_name: str,
              robot_namespace: Optional[str] = None,
              initial_pose: Optional[Pose] = None,
              reference_frame: Optional[str] = None,
              max_retry_attempts: int = 10,
              backoff_time_sec: float = 1.0,
              **kwargs) -> bool:
        """
        Spawn model in gazebo simulator

        Args:
            model_name (str): name of the model.
            robot_namespace (Optional[str]): robot namespace
            initial_pose (Optional[Pose]): initial model spawn pose
            reference_frame (Optional[str]): reference frame
            max_retry_attempts (int): max retry attempts for waiting spawn to complete
            backoff_time_sec (float): backoff time in seconds for spawn to complete
            **kwargs: Arbitrary keyword arguments

        Returns:
            bool: True if model is successfully spawned, otherwise False.
        """
        with AbstractModelSpawner._model_name_lock:
            if model_name not in AbstractModelSpawner._model_names:
                robot_namespace = robot_namespace or "/{}".format(model_name)
                reference_frame = reference_frame or ''
                initial_pose = initial_pose or Pose()
                self._spawn(model_name=model_name,
                            robot_namespace=robot_namespace,
                            initial_pose=initial_pose,
                            reference_frame=reference_frame,
                            **kwargs)
                if self._should_validate_spawn:
                    ROSUtil.wait_for_model_spawn(model_name=model_name,
                                                 max_retry_attempts=max_retry_attempts,
                                                 backoff_time_sec=backoff_time_sec)
                AbstractModelSpawner._model_names.add(model_name)
                return True
            else:
                rospy.loginfo("[AbstractModelSpawner]: {} cannot be spawned "
                              "again without be deleted".format(model_name))
            return False

    def delete(self,
               model_name: str,
               max_retry_attempts: int = 10,
               backoff_time_sec: float = 1.0,
               **kwargs) -> bool:
        """
        Delete model from gazebo simulator

        Args:
            model_name (str): name of the model.
            max_retry_attempts (int): max retry attempts for waiting delete to complete
            backoff_time_sec (float): backoff time in seconds for delete to complete
            **kwargs: Arbitrary keyword arguments

        Returns:
            bool: True if model is successfully spawned, otherwise False.
        """
        with AbstractModelSpawner._model_name_lock:
            if model_name in AbstractModelSpawner._model_names:
                self._delete(model_name=model_name,
                             **kwargs)
                if self._should_validate_delete:
                    ROSUtil.wait_for_model_delete(model_name=model_name,
                                                  max_retry_attempts=max_retry_attempts,
                                                  backoff_time_sec=backoff_time_sec)
                AbstractModelSpawner._model_names.discard(model_name)
                return True
            else:
                rospy.loginfo(
                    "[AbstractModelSpawner]: model ({}) does not exist and "
                    "cannot be deleted".format(model_name))
            return False

    @abc.abstractmethod
    def _spawn(self, model_name: str,
               robot_namespace: str,
               initial_pose: Pose,
               reference_frame: str,
               **kwargs) -> None:
        """
        Spawn model in gazebo simulator

        Args:
            model_name (str): name of the model.
            robot_namespace (str): robot namespace
            initial_pose (Pose): model spawn pose
            reference_frame (str): reference frame
            **kwargs: Arbitrary keyword arguments

        Raises:
            NotImplementedError
        """
        raise NotImplementedError('spawn is not implemented')

    @abc.abstractmethod
    def _delete(self, model_name: str, **kwargs) -> None:
        """
        Delete model from gazebo simulator

        Args:
            model_name (str): name of the model.
            **kwargs: Arbitrary keyword arguments

        Raises:
            NotImplementedError
        """
        raise NotImplementedError('delete is not implemented')
