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

from deepsim.core.twist import Twist
from deepsim.core.vector3 import Vector3

from geometry_msgs.msg import Twist as ROSTwist


class TwistTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        twist = Twist()
        assert twist.linear == Vector3()
        assert twist.angular == Vector3()

        linear = Vector3(1.0, 2.0, 3.0)
        angular = Vector3(2.0, 3.0, 4.0)

        twist2 = Twist(linear=linear,
                       angular=angular)
        assert twist2.linear == linear
        assert twist2.linear is not linear
        assert twist2.angular == angular
        assert twist2.angular is not angular

    def test_setters(self):
        twist = Twist()
        assert twist.linear == Vector3()
        assert twist.angular == Vector3()

        linear = Vector3(1.0, 2.0, 3.0)
        angular = Vector3(2.0, 3.0, 4.0)

        twist.linear = linear
        twist.angular = angular

        assert twist.linear == linear
        assert twist.linear is not linear
        assert twist.angular == angular
        assert twist.angular is not angular

    def test_to_ros(self):
        linear = Vector3(1.0, 2.0, 3.0)
        angular = Vector3(2.0, 3.0, 4.0)

        ros_twist = ROSTwist()
        ros_twist.linear = linear.to_ros()
        ros_twist.angular = angular.to_ros()

        twist = Twist(linear=linear,
                      angular=angular)
        assert twist.to_ros() == ros_twist

    def test_from_ros(self):
        linear = Vector3(1.0, 2.0, 3.0)
        angular = Vector3(2.0, 3.0, 4.0)

        ros_twist = ROSTwist()
        ros_twist.linear = linear.to_ros()
        ros_twist.angular = angular.to_ros()

        expected_twist = Twist(linear=linear,
                               angular=angular)
        assert Twist.from_ros(ros_twist) == expected_twist

    def test_copy(self):
        linear = Vector3(1.0, 2.0, 3.0)
        angular = Vector3(2.0, 3.0, 4.0)

        twist = Twist(linear=linear,
                      angular=angular)

        twist_copy = twist.copy()
        assert twist_copy == twist
        assert twist_copy is not twist
        assert twist_copy.linear == twist.linear
        assert twist_copy.linear is not twist.linear
        assert twist_copy.angular == twist.angular
        assert twist_copy.angular is not twist.angular

    def test_add(self):
        linear = Vector3(1.0, 2.0, 3.0)
        angular = Vector3(2.0, 3.0, 4.0)

        linear2 = Vector3(2.0, 3.0, 4.0)
        angular2 = Vector3(3.0, 4.0, 5.0)

        twist = Twist(linear=linear,
                      angular=angular)
        twist2 = Twist(linear=linear2,
                       angular=angular2)

        expected_twist = Twist(linear=Vector3(3.0, 5.0, 7.0),
                               angular=Vector3(5.0, 7.0, 9.0))
        assert twist + twist2 == expected_twist

    def test_sub(self):
        linear = Vector3(2.0, 3.0, 4.0)
        angular = Vector3(3.0, 4.0, 5.0)

        linear2 = Vector3(1.0, 2.0, 3.0)
        angular2 = Vector3(2.0, 3.0, 4.0)

        twist = Twist(linear=linear,
                      angular=angular)
        twist2 = Twist(linear=linear2,
                       angular=angular2)

        expected_twist = Twist(linear=Vector3(1.0, 1.0, 1.0),
                               angular=Vector3(1.0, 1.0, 1.0))
        assert twist - twist2 == expected_twist

    def test_eq(self):
        linear = Vector3(1.0, 2.0, 3.0)
        angular = Vector3(2.0, 3.0, 4.0)

        twist = Twist(linear=linear,
                      angular=angular)

        twist2 = Twist(linear=linear,
                       angular=angular)

        assert twist == twist2

        linear2 = Vector3(2.0, 3.0, 4.0)
        angular2 = Vector3(3.0, 4.0, 5.0)
        twist2 = Twist(linear=linear2,
                       angular=angular2)

        assert twist != twist2

