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
"""A class for ROS utility."""
import time
from typing import Optional

from deepsim.sim_trackers.trackers.get_model_state_tracker import GetModelStateTracker
from deepsim.exception import DeepSimException

import rospy
import rospkg
import rosnode


class ROSUtil:
    """
    ROS Utility Class
    """
    rospack = rospkg.RosPack()

    @staticmethod
    def is_model_spawned(model_name: str) -> bool:
        """
        Returns True if model is spawned, otherwise False.

        Args:
            model_name (str): the name of the model to check.

        Returns:
            bool: True if model is spawned, otherwise False.
        """
        try:
            GetModelStateTracker.get_instance().get_model_state(name=model_name,
                                                                blocking=True)
            return True
        except DeepSimException:
            return False

    @staticmethod
    def wait_for_model_spawn(model_name: str,
                             max_retry_attempts: int = 10,
                             backoff_time_sec: float = 1.0) -> None:
        """Wait for model spawn to complete

        Args:
            model_name (str): the name of the model spawning.
            max_retry_attempts (int): max retry attempts for waiting spawn to complete
            backoff_time_sec (float): backoff time in seconds for spawn to complete

        Raises:
            DeepRacerEnvException: exception if spawn fails to complete
        """
        try_count = 0
        while not ROSUtil.is_model_spawned(model_name):
            try_count += 1
            if try_count > max_retry_attempts:
                raise DeepSimException("[ROSUtil]: Failed to spawn the model {}".format(model_name))
            rospy.loginfo("[ROSUtil]: model {} is in processing of "
                          "spawning".format(model_name))
            time.sleep(backoff_time_sec)
        rospy.loginfo("[ROSUtil]: spawn {} completed".format(model_name))

    @staticmethod
    def wait_for_model_delete(model_name: str,
                              max_retry_attempts: int = 10,
                              backoff_time_sec: float = 1.0) -> None:
        """Wait for delete to complete

        Args:
            model_name (str): the name of the model deleting.
            max_retry_attempts (int): max retry attempts for waiting delete to complete
            backoff_time_sec (float): backoff time in seconds for delete to complete

        Raises:
            DeepRacerEnvException: exception if delete fails to complete
        """
        try_count = 0
        while ROSUtil.is_model_spawned(model_name):
            rospy.loginfo("[ROSUtil]: model {} is in processing of "
                          "deleting".format(model_name))
            time.sleep(backoff_time_sec)
            try_count += 1
            if try_count > max_retry_attempts:
                raise DeepSimException("[ROSUtil]: delete {} failed".format(model_name))
        rospy.loginfo("[ROSUtil]: delete {} completed".format(model_name))

    @staticmethod
    def is_ros_node_alive(node_name: str) -> bool:
        """Return whether ros node is alive or not

        Args:
            node_name (str): ros node name

        Returns:
            bool: True is ros node is alive, False otherwise.
        """
        if node_name in rosnode.get_node_names():
            return True
        return False

    @staticmethod
    def wait_for_rosnode(alive_nodes: Optional[list] = None,
                         dead_nodes: Optional[list] = None,
                         max_retry_attempts: int = 10,
                         backoff_time_sec: float = 1.0) -> None:
        """Wait for starting/killing ros node to complete

        Args:
            alive_nodes(Optional[list]): list of alive nodes which should be started
            dead_nodes(Optional[list]): list of dead nodes which should be killed
            max_retry_attempts (int): max retry attempts for waiting ROS node check to complete
            backoff_time_sec (float): backoff time in seconds for ROS node check to complete

        Raises:
            DeepRacerEnvException: exception if starting/killing ros node fails to complete
        """
        try_count = 0
        alive_nodes = alive_nodes or list()
        dead_nodes = dead_nodes or list()
        while True:
            if all([ROSUtil.is_ros_node_alive(node) for node in alive_nodes]) and \
                    all([not ROSUtil.is_ros_node_alive(node) for node in dead_nodes]):
                break
            try_count += 1
            if try_count > max_retry_attempts:
                raise DeepSimException(
                    "[ROSUtil]: wait_for_rosnode starting ros node {} "
                    "or killing ros node {} failed".format(alive_nodes, dead_nodes))
            rospy.loginfo("[ROSUtil]: wait_for_rosnode starting ros node {} "
                          "or killing ros node {}".format(alive_nodes, dead_nodes))
            time.sleep(backoff_time_sec)
