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

from deepsim.math.link_state import LinkState
from deepsim.math.pose import Pose
from deepsim.math.twist import Twist
from deepsim.math.point import Point
from deepsim.math.quaternion import Quaternion
from deepsim.math.vector3 import Vector3

from gazebo_msgs.msg import LinkState as ROSLinkState


myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class LinkStateTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        link_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist,
                               reference_frame=reference_frame)

        assert link_state.link_name == link_name
        assert link_state.pose == pose
        assert link_state.pose is not pose
        assert link_state.twist == twist
        assert link_state.twist is not twist
        assert link_state.reference_frame == reference_frame

    def test_setters(self):
        link_state = LinkState()
        assert link_state.link_name is None
        assert link_state.reference_frame == ''
        assert link_state.pose == Pose()
        assert link_state.twist == Twist()

        link_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        link_state.link_name = link_name
        link_state.reference_frame = reference_frame
        link_state.pose = pose
        link_state.twist = twist

        assert link_state.link_name == link_name
        assert link_state.pose == pose
        assert link_state.pose is not pose
        assert link_state.twist == twist
        assert link_state.twist is not twist
        assert link_state.reference_frame == reference_frame

    def test_to_ros(self):
        link_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        ros_link_state = ROSLinkState()
        ros_link_state.link_name = link_name
        ros_link_state.reference_frame = reference_frame
        ros_link_state.pose = pose.to_ros()
        ros_link_state.twist = twist.to_ros()

        link_state = LinkState(link_name=link_name,
                               reference_frame=reference_frame,
                               pose=pose,
                               twist=twist)

        assert link_state.to_ros() == ros_link_state

    def test_from_ros(self):
        link_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        ros_link_state = ROSLinkState()
        ros_link_state.link_name = link_name
        ros_link_state.reference_frame = reference_frame
        ros_link_state.pose = pose.to_ros()
        ros_link_state.twist = twist.to_ros()

        link_state = LinkState.from_ros(ros_link_state)

        assert link_state == LinkState(link_name=link_name,
                                       reference_frame=reference_frame,
                                       pose=pose,
                                       twist=twist)

    def test_copy(self):
        link_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist,
                               reference_frame=reference_frame)

        link_state_copy = link_state.copy()
        assert link_state_copy.link_name == link_state.link_name
        assert link_state_copy.pose == link_state.pose
        assert link_state_copy.pose is not link_state.pose
        assert link_state_copy.twist == link_state.twist
        assert link_state_copy.twist is not link_state.twist
        assert link_state_copy.reference_frame == link_state.reference_frame

    def test_eq(self):
        link_name = myself()
        reference_frame = myself() + "_reference"
        pose = Pose(position=Point(0.1, 0.2, 0.3),
                    orientation=Quaternion(0.2, 0.3, 0.4, 0.5))
        twist = Twist(linear=Vector3(0.3, 0.4, 0.5),
                      angular=Vector3(0.4, 0.5, 0.6))

        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist,
                               reference_frame=reference_frame)
        link_state2 = LinkState(link_name=link_name,
                                pose=pose,
                                twist=twist,
                                reference_frame=reference_frame)

        assert link_state == link_state2

        pose2 = Pose(position=Point(0.2, 0.3, 0.4),
                     orientation=Quaternion(0.3, 0.4, 0.5, 0.6))

        link_state3 = LinkState(link_name=link_name,
                                pose=pose2,
                                twist=twist,
                                reference_frame=reference_frame)
        assert link_state != link_state3
