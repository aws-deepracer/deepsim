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
"""This test Singleton class handling gazebo model spawn and delete service"""
from unittest import mock, TestCase
from unittest.mock import patch, MagicMock, call, ANY

from deepsim.gazebo.constants import GazeboServiceName
from deepsim.spawners.gazebo_model_spawner import GazeboModelSpawner
from deepsim.core.pose import Pose

from gazebo_msgs.srv import SpawnModel, DeleteModel


@patch("deepsim.spawners.gazebo_model_spawner.rospy")
@patch("deepsim.spawners.gazebo_model_spawner.ServiceProxyWrapper")
class GazeboModelTest(TestCase):
    def setUp(self) -> None:
        self.model_name = "agent"
        self.model_xml = "agent_xml"
        self.robot_namespace = "agent_namespace"
        self.initial_pose = Pose()
        self.reference_frame = "agent_reference_frame"

    def test_get_instance(self, service_proxy_wrapper_mock, rospy_mock) -> None:
        """test GazeboModel Singleton class get_instance method

        Args:
            service_proxy_wrapper_mock (Mock): service_proxy_wrapper mock
            rospy_mock (Mock): rospy mock
        """
        _ = GazeboModelSpawner(is_singleton=False)
        rospy_mock.wait_for_service.assert_has_calls([
            call(GazeboServiceName.SPAWN_URDF_MODEL),
            call(GazeboServiceName.SPAWN_SDF_MODEL),
            call(GazeboServiceName.DELETE_MODEL)])
        service_proxy_wrapper_mock.assert_has_calls([
            call(GazeboServiceName.SPAWN_URDF_MODEL,
                 SpawnModel),
            call(GazeboServiceName.SPAWN_SDF_MODEL,
                 SpawnModel),
            call(GazeboServiceName.DELETE_MODEL,
                 DeleteModel)])

    def test_spawn_sdf(self, service_proxy_wrapper_mock, rospy_mock) -> None:
        """test GazeboModel spawn sdf model

        Args:
            service_proxy_wrapper_mock (Mock): service_proxy_wrapper mock
            rospy_mock (Mock): rospy mock
        """
        gazebo_model_spawner = GazeboModelSpawner(is_singleton=False)
        gazebo_model_spawner.spawn_sdf(self.model_name,
                                       self.model_xml,
                                       self.robot_namespace,
                                       self.initial_pose,
                                       self.reference_frame)
        gazebo_model_spawner._spawn_sdf_model.assert_called_once_with(
            self.model_name,
            self.model_xml,
            self.robot_namespace,
            self.initial_pose.to_ros(),
            self.reference_frame)

    def test_spawn_urdf(self, service_proxy_wrapper_mock, rospy_mock) -> None:
        """test GazeboModel spawn urdf model

        Args:
            service_proxy_wrapper_mock (Mock): service_proxy_wrapper mock
            rospy_mock (Mock): rospy mock
        """
        gazebo_model_spawner = GazeboModelSpawner(is_singleton=False)
        gazebo_model_spawner.spawn_urdf(self.model_name,
                                        self.model_xml,
                                        self.robot_namespace,
                                        self.initial_pose,
                                        self.reference_frame)
        gazebo_model_spawner._spawn_urdf_model.assert_called_once_with(
            self.model_name,
            self.model_xml,
            self.robot_namespace,
            self.initial_pose.to_ros(),
            self.reference_frame)

    def test_delete(self, service_proxy_wrapper_mock, rospy_mock) -> None:
        """test GazeboModel delete model

        Args:
            service_proxy_wrapper_mock (Mock): service_proxy_wrapper mock
            rospy_mock (Mock): rospy mock
        """
        gazebo_model_spawner = GazeboModelSpawner(is_singleton=False)
        gazebo_model_spawner.delete(self.model_name)
        gazebo_model_spawner._delete_model.assert_called_once_with(self.model_name)
