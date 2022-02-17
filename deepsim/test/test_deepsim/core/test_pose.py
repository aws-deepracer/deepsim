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

from deepsim.core.pose import Pose
from deepsim.core.point import Point
from deepsim.core.quaternion import Quaternion

from geometry_msgs.msg import Pose as ROSPose


class PoseTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        pose = Pose()
        assert pose.position == Point()
        assert pose.orientation == Quaternion()

        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)
        pose = Pose(position=position,
                    orientation=orientation)

        assert pose.position == position
        assert pose.position is not position
        assert pose.orientation == orientation
        assert pose.orientation is not orientation

    def test_setters(self):
        pose = Pose()
        assert pose.position == Point()
        assert pose.orientation == Quaternion()

        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)

        pose.position = position
        pose.orientation = orientation

        assert pose.position == position
        assert pose.position is not position
        assert pose.orientation == orientation
        assert pose.orientation is not orientation

    def test_to_ros(self):
        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)
        pose = Pose(position=position,
                    orientation=orientation)

        ros_pose = ROSPose()
        ros_pose.position = position.to_ros()
        ros_pose.orientation = orientation.to_ros()

        assert pose.to_ros() == ros_pose

    def test_from_ros(self):
        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)

        ros_pose = ROSPose()
        ros_pose.position = position.to_ros()
        ros_pose.orientation = orientation.to_ros()

        pose = Pose.from_ros(ros_pose)

        assert pose == Pose(position=position,
                            orientation=orientation)

    def test_copy(self):
        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)
        pose = Pose(position=position,
                    orientation=orientation)

        pose_copy = pose.copy()
        assert pose_copy == pose
        assert pose_copy.position == pose.position
        assert pose_copy.position is not pose.position
        assert pose_copy.orientation == pose.orientation
        assert pose_copy.orientation is not pose.orientation

    def test_add(self):
        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)
        pose = Pose(position=position,
                    orientation=orientation)

        position2 = Point(2.0, 3.0, 4.0)
        orientation2 = Quaternion(3.0, 4.0, 5.0, 6.0)
        pose2 = Pose(position=position2,
                     orientation=orientation2)

        expected_pose = Pose(position=position + position2.rotate(orientation),
                             orientation=orientation * orientation2)
        assert pose + pose2 == expected_pose

    def test_sub(self):
        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)
        pose = Pose(position=position,
                    orientation=orientation)

        position2 = Point(2.0, 3.0, 4.0)
        orientation2 = Quaternion(3.0, 4.0, 5.0, 6.0)
        pose2 = Pose(position=position2,
                     orientation=orientation2)

        expected_pose = Pose(position=position - position2.rotate(orientation),
                             orientation=orientation * orientation2.inverse())
        assert pose - pose2 == expected_pose

    def test_iadd(self):
        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)
        pose = Pose(position=position,
                    orientation=orientation)

        position2 = Point(2.0, 3.0, 4.0)
        orientation2 = Quaternion(3.0, 4.0, 5.0, 6.0)
        pose2 = Pose(position=position2,
                     orientation=orientation2)

        expected_pose = Pose(position=position + position2.rotate(orientation),
                             orientation=orientation * orientation2)
        pose += pose2
        assert pose == expected_pose

    def test_isub(self):
        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)
        pose = Pose(position=position,
                    orientation=orientation)

        position2 = Point(2.0, 3.0, 4.0)
        orientation2 = Quaternion(3.0, 4.0, 5.0, 6.0)
        pose2 = Pose(position=position2,
                     orientation=orientation2)

        expected_pose = Pose(position=position - position2.rotate(orientation),
                             orientation=orientation * orientation2.inverse())

        pose -= pose2
        assert pose == expected_pose

    def test_neg(self):
        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)
        pose = Pose(position=position,
                    orientation=orientation)
        pose_neg = -pose

        assert pose_neg.position == -position
        assert pose_neg.orientation == -orientation

    def test_eq(self):
        position = Point(1.0, 2.0, 3.0)
        orientation = Quaternion(2.0, 3.0, 4.0, 5.0)
        pose = Pose(position=position,
                    orientation=orientation)

        pose2 = Pose(position=position,
                     orientation=orientation)
        assert pose == pose2
        assert pose.position == pose2.position
        assert pose.position is not pose2.position
        assert pose.orientation == pose2.orientation
        assert pose.orientation is not pose2.orientation

        position2 = Point(2.0, 2.0, 3.0)
        orientation2 = Quaternion(3.0, 3.0, 4.0, 5.0)
        pose2 = Pose(position=position2,
                     orientation=orientation2)
        assert pose != pose2
