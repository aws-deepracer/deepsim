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

from deepsim.core.model_state import ModelState
from deepsim.core.pose import Pose
from deepsim.core.twist import Twist
from deepsim.core.point import Point
from deepsim.core.quaternion import Quaternion
from deepsim.core.vector3 import Vector3

from gazebo_msgs.msg import ModelState as ROSModelState


myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class ModelStateTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        model_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        model_state = ModelState(model_name=model_name,
                                 pose=pose,
                                 twist=twist,
                                 reference_frame=reference_frame)

        assert model_state.model_name == model_name
        assert model_state.pose == pose
        assert model_state.pose is not pose
        assert model_state.twist == twist
        assert model_state.twist is not twist
        assert model_state.reference_frame == reference_frame

    def test_setters(self):
        model_state = ModelState()
        assert model_state.model_name is None
        assert model_state.reference_frame == ''
        assert model_state.pose == Pose()
        assert model_state.twist == Twist()

        model_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        model_state.model_name = model_name
        model_state.reference_frame = reference_frame
        model_state.pose = pose
        model_state.twist = twist

        assert model_state.model_name == model_name
        assert model_state.pose == pose
        assert model_state.pose is not pose
        assert model_state.twist == twist
        assert model_state.twist is not twist
        assert model_state.reference_frame == reference_frame

    def test_to_ros(self):
        model_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        ros_model_state = ROSModelState()
        ros_model_state.model_name = model_name
        ros_model_state.reference_frame = reference_frame
        ros_model_state.pose = pose.to_ros()
        ros_model_state.twist = twist.to_ros()

        model_state = ModelState(model_name=model_name,
                                 reference_frame=reference_frame,
                                 pose=pose,
                                 twist=twist)

        assert ros_model_state == model_state.to_ros()

    def test_from_ros(self):
        model_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        ros_model_state = ROSModelState()
        ros_model_state.model_name = model_name
        ros_model_state.reference_frame = reference_frame
        ros_model_state.pose = pose.to_ros()
        ros_model_state.twist = twist.to_ros()

        model_state = ModelState.from_ros(ros_model_state)

        assert model_state == ModelState(model_name=model_name,
                                         reference_frame=reference_frame,
                                         pose=pose,
                                         twist=twist)

    def test_copy(self):
        model_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        model_state = ModelState(model_name=model_name,
                                 pose=pose,
                                 twist=twist,
                                 reference_frame=reference_frame)

        model_state_copy = model_state.copy()
        assert model_state_copy.model_name == model_state.model_name
        assert model_state_copy.pose == model_state.pose
        assert model_state_copy.pose is not model_state.pose
        assert model_state_copy.twist == model_state.twist
        assert model_state_copy.twist is not model_state.twist
        assert model_state_copy.reference_frame == model_state.reference_frame

    def test_eq(self):
        model_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        model_state = ModelState(model_name=model_name,
                                 pose=pose,
                                 twist=twist,
                                 reference_frame=reference_frame)
        model_state2 = ModelState(model_name=model_name,
                                  pose=pose,
                                  twist=twist,
                                  reference_frame=reference_frame)

        assert model_state == model_state2

        pose2 = Pose(position=Point(0.2, 0.3, 0.4),
                     orientation=Quaternion(0.3, 0.4, 0.5, 0.6))

        model_state3 = ModelState(model_name=model_name,
                                  pose=pose2,
                                  twist=twist,
                                  reference_frame=reference_frame)
        assert model_state != model_state3
