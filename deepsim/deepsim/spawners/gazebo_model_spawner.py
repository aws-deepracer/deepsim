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
"""A class for Gazebo model spawner."""
from threading import RLock

from deepsim.gazebo.constants import GazeboServiceName
from deepsim.core.pose import Pose
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper

import rospy
from gazebo_msgs.srv import (SpawnModel, DeleteModel,
                             SpawnModelResponse, DeleteModelResponse)


class GazeboModelSpawner(object):
    """GazeboSpawner Singleton class"""
    _instance = None
    _instance_lock = RLock()

    @staticmethod
    def get_instance() -> 'GazeboModelSpawner':
        """
        Method for getting a reference to the GazeboModelSpawner object

        Returns:
            GazeboModelSpawner: GazeboModelSpawner instance
        """
        with GazeboModelSpawner._instance_lock:
            if GazeboModelSpawner._instance is None:
                GazeboModelSpawner()
            return GazeboModelSpawner._instance

    def __init__(self, is_singleton: bool = True):
        """
        Constructor for GazeboModelSpawner

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if GazeboModelSpawner._instance is not None:
                raise RuntimeError("Attempting to construct multiple GazeboModelSpawner")
            GazeboModelSpawner._instance = self
        rospy.wait_for_service(GazeboServiceName.SPAWN_URDF_MODEL)
        rospy.wait_for_service(GazeboServiceName.SPAWN_SDF_MODEL)
        rospy.wait_for_service(GazeboServiceName.DELETE_MODEL)
        self._spawn_urdf_model = ServiceProxyWrapper(GazeboServiceName.SPAWN_URDF_MODEL,
                                                     SpawnModel)
        self._spawn_sdf_model = ServiceProxyWrapper(GazeboServiceName.SPAWN_SDF_MODEL,
                                                    SpawnModel)
        self._delete_model = ServiceProxyWrapper(GazeboServiceName.DELETE_MODEL,
                                                 DeleteModel)

    def spawn_sdf(self,
                  model_name: str,
                  model_xml: str,
                  robot_namespace: str,
                  initial_pose: Pose,
                  reference_frame: str) -> SpawnModelResponse:
        """
        Spawn sdf model

        http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/SpawnModel.html

        Args:
            model_name (string): name of the model to be spawn
            model_xml (string): this should be an urdf or gazebo xml
            robot_namespace (string): spawn robot and all ROS interfaces under this namespace
            initial_pose (Pose): only applied to canonical body
            reference_frame (string): initial_pose is defined relative to the frame of this model/body
                                      if left empty or "world", then gazebo world frame is used
                                      if non-existent model/body is specified, an error is returned
                                      and the model is not spawned

        Returns:
            SpawnModelResponse: response msg
        """
        return self._spawn_sdf_model(model_name,
                                     model_xml,
                                     robot_namespace,
                                     initial_pose.to_ros(),
                                     reference_frame)

    def spawn_urdf(self,
                   model_name: str,
                   model_xml: str,
                   robot_namespace: str,
                   initial_pose: Pose,
                   reference_frame: str) -> SpawnModelResponse:
        """
        Spawn urdf model

        http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/SpawnModel.html

        Args:
            model_name (string): name of the model to be spawn
            model_xml (string): this should be an urdf or gazebo xml
            robot_namespace (string): spawn robot and all ROS interfaces under this namespace
            initial_pose (Pose): only applied to canonical body
            reference_frame (string): initial_pose is defined relative to the frame of this model/body
                                      if left empty or "world", then gazebo world frame is used
                                      if non-existent model/body is specified, an error is returned
                                      and the model is not spawned

        Returns:
            SpawnModelResponse: response msg
        """
        return self._spawn_urdf_model(model_name,
                                      model_xml,
                                      robot_namespace,
                                      initial_pose.to_ros(),
                                      reference_frame)

    def delete(self, model_name: str) -> DeleteModelResponse:
        """
        Delete model

        http://docs.ros.org/en/jade/api/gazebo_msgs/html/srv/DeleteModel.html

        Args:
            model_name (string): name of the Gazebo Model to be deleted

        Returns:
            DeleteModelResponse: response msg
        """
        return self._delete_model(model_name)
