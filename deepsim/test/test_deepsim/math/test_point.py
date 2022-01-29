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

import numpy as np
import math

from deepsim.math.point import Point
from deepsim.math.vector3 import Vector3
from deepsim.math.euler import Euler
from deepsim.math.quaternion import Quaternion

from geometry_msgs.msg import Point as ROSPoint
from shapely.geometry import Point as ShapelyPoint

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class PointTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        # default initialization
        point = Point()
        assert point.x == 0.0 and point.y  == 0.0 and point.z == 0.0

        # initialize with  rgba
        point2 = Point(0.5, 0.6, 0.7)
        assert point2.x == 0.5 and point2.y == 0.6 and point2.z == 0.7

        # initialize with buffer
        point3 = Point(buffer=np.array([0.8, 0.7, 0.6]))
        assert point3.x == 0.8 and point3.y == 0.7 and point3.z == 0.6

        # buffer takes precedent.
        point4 = Point(0.5, 0.6, 0.7,
                       buffer=np.array([0.8, 0.7, 0.6]))
        assert point4.x == 0.8 and point4.y == 0.7 and point4.z == 0.6

        # buffer with smaller size
        with self.assertRaises(ValueError):
            _ = Point(buffer=np.array([0.8, 0.7]))

        point5 = Point(buffer=np.array([0.8, 0.7, 0.6, 0.5]))
        assert point5.x == 0.8 and point5.y == 0.7 and point5.z == 0.6

    def test_setter(self):
        point = Point()
        assert point.x == 0.0 and point.y == 0.0 and point.z == 0.0

        point.x = 0.5
        point.y = 0.6
        point.z = 0.7
        assert point.x == 0.5 and point.y == 0.6 and point.z == 0.7

    def test_rotate_point(self):
        point = Point(1.0, 0.0, 0.0)
        quat = Quaternion.from_euler(Euler(0.0, 0.0, math.pi))
        new_point = Point.rotate_point(point, quat)
        expected_point = Point(-1.0, 0.0, 0.0)
        assert new_point == expected_point

    def test_project(self):
        point = Point(0.5, 0.6, 0.7)
        normal = Vector3(0.0, 2.0, 3.0)
        new_point = Point.project(point, normal)
        expected_point = Point(0.0, 0.5076923076923077, 0.7615384615384614)
        assert new_point == expected_point

    def test_rotate(self):
        point = Point(0.5, 0.6, 0.7)
        quat = Quaternion.from_euler(Euler(1.5, 0.0, 1.5))
        new_point = point.rotate(quat)
        expected_point = Point(0.6895299722260471, 0.4523577415009401, 0.6480130331298246)
        assert new_point == expected_point

        point = Point(1.0, 0.0, 0.0)
        quat = Quaternion.from_euler(Euler(0.0, 0.0, math.pi))
        new_point = point.rotate(quat)
        expected_point = Point(-1.0, 0.0, 0.0)
        assert new_point == expected_point

    def test_rotate_inplace(self):
        point = Point(1.0, 0.0, 0.0)
        quat = Quaternion.from_euler(Euler(0.0, 0.0, math.pi / 2))
        new_point = point.rotate(quat)
        expected_point = Point(0.0, 1.0, 0.0)
        assert new_point == expected_point

    def test_to_ros(self):
        point = Point(2.0, 3.0, 4.0)

        expected_ros_point = ROSPoint()
        expected_ros_point.x = 2.0
        expected_ros_point.y = 3.0
        expected_ros_point.z = 4.0

        assert point.to_ros() == expected_ros_point

    def test_to_list(self):
        point = Point(2.0, 3.0, 4.0)

        expected_point = [2.0, 3.0, 4.0]

        assert point.to_list() == expected_point

    def test_to_numpy(self):
        point = Point(2.0, 3.0, 4.0)

        expected_np_point = np.array([2.0, 3.0, 4.0])

        assert np.all(point.to_numpy() == expected_np_point)

    def test_to_vector(self):
        point = Point(2.0, 3.0, 4.0)

        expected_vector = Vector3(2.0, 3.0, 4.0)

        assert point.to_vector() == expected_vector

    def test_to_shapely(self):
        point = Point(2.0, 3.0, 4.0)

        expected_shapely = ShapelyPoint([2.0, 3.0, 4.0])

        assert point.to_shapely() == expected_shapely

    def test_to_shapely_2d(self):
        point = Point(2.0, 3.0, 4.0)

        expected_shapely = ShapelyPoint([2.0, 3.0])

        assert point.to_shapely_2d() == expected_shapely




    def test_from_ros(self):
        expected_point = Point(2.0, 3.0, 4.0)

        ros_point = ROSPoint()
        ros_point.x = 2.0
        ros_point.y = 3.0
        ros_point.z = 4.0

        assert Point.from_ros(ros_point) == expected_point

    def test_from_list(self):
        expected_point = Point(2.0, 3.0, 4.0)

        point_in_list = [2.0, 3.0, 4.0]

        assert Point.from_list(point_in_list) == expected_point

    def test_from_numpy(self):
        expected_point = Point(2.0, 3.0, 4.0)

        np_point = np.array([2.0, 3.0, 4.0])

        assert Point.from_numpy(np_point) == expected_point

    def test_from_vector(self):
        expected_point = Point(2.0, 3.0, 4.0)

        vector = Vector3(2.0, 3.0, 4.0)

        assert Point.from_vector(vector) == expected_point

    def test_from_shapely(self):
        expected_point = Point(2.0, 3.0, 4.0)

        shapely_point = ShapelyPoint([2.0, 3.0, 4.0])

        assert Point.from_shapely(shapely_point) == expected_point

    def test_get_angle_in_2d_rad(self):
        point1 = Point(1.0, 0.0, 0.0)
        point2 = Point(0.0, 1.0, 0.0)
        angle = Point.get_angle_in_2d_rad(point1, point2)
        assert angle == 2.356194490192345

        point1 = Point(0.0, 0.0, 0.0)
        point2 = Point(0.0, 1.0, 0.0)
        angle = Point.get_angle_in_2d_rad(point1, point2)
        assert angle == math.pi / 2.0

    def test_copy(self):
        point = Point(1.0, 2.0, 3.0)
        point_copy = point.copy()
        assert point == point_copy

    def test_add(self):
        point = Point(1.0, 2.0, 3.0)
        other = Point(2.0, 3.0, 4.0)

        assert point + other == Point(3.0, 5.0, 7.0)

        other_vec = Vector3(2.0, 3.0, 4.0)
        assert point + other_vec == Point(3.0, 5.0, 7.0)

    def test_sub(self):
        point = Point(2.0, 3.0, 4.0)
        other = Point(1.0, 2.0, 3.0)

        assert point - other == Point(1.0, 1.0, 1.0)

        other_vec = Vector3(1.0, 2.0, 3.0)
        assert point - other_vec == Point(1.0, 1.0, 1.0)

    def test_mul(self):
        point = Point(2.0, 3.0, 4.0)
        other = Point(1.0, 2.0, 3.0)

        assert point * other == Point(2.0, 6.0, 12.0)

        other_vec = Vector3(1.0, 2.0, 3.0)
        assert point * other_vec == Point(2.0, 6.0, 12.0)

    def test_rmul(self):
        point = Point(2.0, 3.0, 4.0)

        assert 10 * point == Point(20.0, 30.0, 40.0)
        assert 5.0 * point == Point(10.0, 15.0, 20.0)
        new_point = Euler(0.0, 0.0, 50.0).to_quaternion() * point
        assert new_point == Point(2.717056618096013, 2.3701483780684827, 4.0)

        point = Point(1.0, 0.0, 0.0)
        new_point = Euler(0.0, 0.0, math.pi / 2.0).to_quaternion() * point
        assert new_point == Point(0.0, 1.0, 0.0)

    def test_truediv(self):
        point = Point(2.0, 3.0, 4.0)

        assert point / 2.0 == Point(1.0, 1.5, 2.0)
        assert point / 2 == Point(1.0, 1.5, 2.0)

    def test_iadd(self):
        point = Point(2.0, 3.0, 4.0)

        vector = Vector3(1.0, 2.0, 3.0)

        point += vector
        assert point == Point(3.0, 5.0, 7.0)

    def test_isub(self):
        point = Point(2.0, 3.0, 4.0)

        vector = Vector3(1.0, 2.0, 3.0)

        point -= vector
        assert point == Point(1.0, 1.0, 1.0)

    def test_imul(self):
        point = Point(2.0, 3.0, 4.0)
        other = Vector3(1.0, 2.0, 3.0)

        point *= other
        assert point == Point(2.0, 6.0, 12.0)

        point = Point(2.0, 3.0, 4.0)
        other = Point(1.0, 2.0, 3.0)

        point *= other
        assert point == Point(2.0, 6.0, 12.0)

        point = Point(2.0, 3.0, 4.0)
        other = 2.0

        point *= other
        assert point == Point(4.0, 6.0, 8.0)

        point = Point(2.0, 3.0, 4.0)
        other = 2

        point *= other
        assert point == Point(4.0, 6.0, 8.0)

    def test_idiv(self):
        point = Point(2.0, 3.0, 4.0)
        other = 2.0

        point /= other
        assert point == Point(1.0, 1.5, 2.0)

        point = Point(2.0, 3.0, 4.0)
        other = 2

        point /= other
        assert point == Point(1.0, 1.5, 2.0)

    def test_neg(self):
        point = Point(2.0, 3.0, 4.0)
        assert -point == Point(-2.0, -3.0, -4.0)

    def test_iter(self):
        point = Point(2.0, 3.0, 4.0)
        for idx, component in enumerate(point):
            if idx == 0:
                assert component == 2.0
            elif idx == 1:
                assert component == 3.0
            elif idx == 2:
                assert component == 4.0

    def test_eq(self):
        point = Point(2.0, 3.0, 4.0)
        other = Point(2.0, 3.0, 4.0)
        assert point == other

        other = Point(2.0, 3.0, 5.0)
        assert point != other

    def test_getitem(self):
        point = Point(2.0, 3.0, 4.0)
        assert point[0] == 2.0
        assert point[1] == 3.0
        assert point[2] == 4.0

    def test_setitem(self):
        point = Point()
        assert point[0] == 0.0 and point[1] == 0.0 and point[2] == 0.0

        point[0] = 2.0
        point[1] = 3.0
        point[2] = 4.0

        assert point[0] == 2.0
        assert point[1] == 3.0
        assert point[2] == 4.0
