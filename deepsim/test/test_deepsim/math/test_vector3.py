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
from deepsim.gazebo.constants import GazeboWorld

from geometry_msgs.msg import Vector3 as ROSVector3
from shapely.geometry import Point as ShapelyPoint

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class Vector3Test(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        # default initialization
        vector = Vector3()
        assert vector.x == 0.0 and vector.y  == 0.0 and vector.z == 0.0

        # initialize with  rgba
        vector2 = Vector3(0.5, 0.6, 0.7)
        assert vector2.x == 0.5 and vector2.y == 0.6 and vector2.z == 0.7

        # initialize with buffer
        vector3 = Vector3(buffer=np.array([0.8, 0.7, 0.6]))
        assert vector3.x == 0.8 and vector3.y == 0.7 and vector3.z == 0.6

        # buffer takes precedent.
        vector4 = Vector3(0.5, 0.6, 0.7,
                          buffer=np.array([0.8, 0.7, 0.6]))
        assert vector4.x == 0.8 and vector4.y == 0.7 and vector4.z == 0.6

        # buffer with smaller size
        with self.assertRaises(ValueError):
            _ = Vector3(buffer=np.array([0.8, 0.7]))

        vector5 = Vector3(buffer=np.array([0.8, 0.7, 0.6, 0.5]))
        assert vector5.x == 0.8 and vector5.y == 0.7 and vector5.z == 0.6

    def test_one(self):
        assert Vector3.one() == Vector3(1.0, 1.0, 1.0)

    def test_zero(self):
        assert Vector3.zero() == Vector3(0.0, 0.0, 0.0)

    def test_static_vectors(self):
        assert Vector3.forward() == GazeboWorld.FORWARD.vector
        assert Vector3.back() == GazeboWorld.BACK.vector
        assert Vector3.left() == GazeboWorld.LEFT.vector
        assert Vector3.right() == GazeboWorld.RIGHT.vector
        assert Vector3.up() == GazeboWorld.UP.vector
        assert Vector3.down() == GazeboWorld.DOWN.vector

    def test_lerp(self):
        a = Vector3(1.0, 1.0, 0.0)
        b = Vector3(2.0, 2.0, 0.0)
        ret_vec = Vector3.lerp(a, b, 0.5)
        expected_vec = Vector3(1.5, 1.5, 0.0)
        assert ret_vec == expected_vec

    def test_lerp_with_points(self):
        a = Point(1.0, 1.0, 0.0)
        b = Vector3(2.0, 2.0, 0.0)
        ret_vec = Vector3.lerp(a, b, 0.5)
        expected_vec = Vector3(1.5, 1.5, 0.0)
        assert ret_vec == expected_vec

        a = Vector3(1.0, 1.0, 0.0)
        b = Point(2.0, 2.0, 0.0)
        ret_vec = Vector3.lerp(a, b, 0.5)
        expected_vec = Vector3(1.5, 1.5, 0.0)
        assert ret_vec == expected_vec

        a = Point(1.0, 1.0, 0.0)
        b = Point(2.0, 2.0, 0.0)
        ret_vec = Vector3.lerp(a, b, 0.5)
        expected_vec = Vector3(1.5, 1.5, 0.0)
        assert ret_vec == expected_vec

    def test_slerp(self):
        a = Vector3(1.0, 0.0, 0.0)
        b = Vector3(0.0, 1.0, 0.0)
        ret_vec = Vector3.slerp(a, b, 0.5)
        expected_vec = Vector3(0.707107, 0.707107, 0.0)
        assert ret_vec == expected_vec

    def test_slerp_with_points(self):
        a = Point(1.0, 0.0, 0.0)
        b = Vector3(0.0, 1.0, 0.0)
        ret_vec = Vector3.slerp(a, b, 0.5)
        expected_vec = Vector3(0.707107, 0.707107, 0.0)
        assert ret_vec == expected_vec

        a = Vector3(1.0, 0.0, 0.0)
        b = Point(0.0, 1.0, 0.0)
        ret_vec = Vector3.slerp(a, b, 0.5)
        expected_vec = Vector3(0.707107, 0.707107, 0.0)
        assert ret_vec == expected_vec

        a = Point(1.0, 0.0, 0.0)
        b = Point(0.0, 1.0, 0.0)
        ret_vec = Vector3.slerp(a, b, 0.5)
        expected_vec = Vector3(0.707107, 0.707107, 0.0)
        assert ret_vec == expected_vec

    def test_setter(self):
        vector = Vector3()
        assert vector.x == 0.0 and vector.y == 0.0 and vector.z == 0.0

        vector.x = 0.5
        vector.y = 0.6
        vector.z = 0.7
        assert vector.x == 0.5 and vector.y == 0.6 and vector.z == 0.7

    def test_sqr_magnitude(self):
        vector = Vector3(2.0, 0.0, 0.0)
        assert vector.sqr_magnitude == 4.0

    def test_magnitude(self):
        vector = Vector3(2.0, 0.0, 0.0)
        assert vector.magnitude == 2.0

    def test_rotate_vector(self):
        vector = Vector3(1.0, 0.0, 0.0)
        quat = Quaternion.from_euler(Euler(0.0, 0.0, math.pi))
        new_vector = Vector3.rotate_vector(vector, quat)
        expected_vector = Vector3(-1.0, 0.0, 0.0)
        assert new_vector == expected_vector

    def test_project(self):
        vector = Vector3(0.5, 0.6, 0.7)
        normal = Vector3(0.0, 2.0, 3.0)
        new_vector = Vector3.project(vector, normal)
        expected_vector = Vector3(0.0, 0.5076923076923077, 0.7615384615384614)
        assert new_vector == expected_vector

    def test_rotate(self):
        vector = Vector3(0.5, 0.6, 0.7)
        quat = Quaternion.from_euler(Euler(1.5, 0.0, 1.5))
        new_vector = vector.rotate(quat)
        expected_vector = Vector3(0.6895299722260471, 0.4523577415009401, 0.6480130331298246)
        assert new_vector == expected_vector

        vector = Vector3(1.0, 0.0, 0.0)
        quat = Quaternion.from_euler(Euler(0.0, 0.0, math.pi))
        new_vector = vector.rotate(quat)
        expected_vector = Vector3(-1.0, 0.0, 0.0)
        assert new_vector == expected_vector

    def test_rotate_inplace(self):
        vector = Vector3(1.0, 0.0, 0.0)
        quat = Quaternion.from_euler(Euler(0.0, 0.0, math.pi / 2))
        new_vector = vector.rotate(quat)
        expected_vector = Vector3(0.0, 1.0, 0.0)
        assert new_vector == expected_vector

    def test_to_ros(self):
        vector = Vector3(2.0, 3.0, 4.0)

        expected_ros_vector = ROSVector3()
        expected_ros_vector.x = 2.0
        expected_ros_vector.y = 3.0
        expected_ros_vector.z = 4.0

        assert vector.to_ros() == expected_ros_vector

    def test_to_list(self):
        vector = Vector3(2.0, 3.0, 4.0)

        expected_vector = [2.0, 3.0, 4.0]

        assert vector.to_list() == expected_vector

    def test_to_numpy(self):
        vector = Vector3(2.0, 3.0, 4.0)

        expected_np_vector = np.array([2.0, 3.0, 4.0])

        assert np.all(vector.to_numpy() == expected_np_vector)

    def test_to_point(self):
        vector = Vector3(2.0, 3.0, 4.0)

        expected_point = Point(2.0, 3.0, 4.0)

        assert vector.to_point() == expected_point

    def test_to_shapely(self):
        vector = Vector3(2.0, 3.0, 4.0)

        expected_shapely = ShapelyPoint([2.0, 3.0, 4.0])

        assert vector.to_shapely() == expected_shapely

    def test_to_shapely_2d(self):
        vector = Vector3(2.0, 3.0, 4.0)

        expected_shapely = ShapelyPoint([2.0, 3.0])

        assert vector.to_shapely_2d() == expected_shapely

    def test_from_ros(self):
        expected_vector = Vector3(2.0, 3.0, 4.0)

        ros_vector = ROSVector3()
        ros_vector.x = 2.0
        ros_vector.y = 3.0
        ros_vector.z = 4.0

        assert Vector3.from_ros(ros_vector) == expected_vector

    def test_from_list(self):
        expected_vector = Vector3(2.0, 3.0, 4.0)

        vector_in_list = [2.0, 3.0, 4.0]

        assert Vector3.from_list(vector_in_list) == expected_vector

    def test_from_numpy(self):
        expected_vector = Vector3(2.0, 3.0, 4.0)

        np_vector = np.array([2.0, 3.0, 4.0])

        assert Vector3.from_numpy(np_vector) == expected_vector

    def test_from_point(self):
        expected_vector = Vector3(2.0, 3.0, 4.0)

        point = Point(2.0, 3.0, 4.0)

        assert Vector3.from_point(point) == expected_vector

    def test_from_shapely(self):
        expected_vector = Vector3(2.0, 3.0, 4.0)

        shapely_vector = ShapelyPoint([2.0, 3.0, 4.0])

        assert Vector3.from_shapely(shapely_vector) == expected_vector

    def test_copy(self):
        vector = Vector3(1.0, 2.0, 3.0)
        vector_copy = vector.copy()
        assert vector == vector_copy

    def test_add(self):
        vector = Vector3(1.0, 2.0, 3.0)
        other = Vector3(2.0, 3.0, 4.0)

        assert vector + other == Vector3(3.0, 5.0, 7.0)

        other_point = Point(2.0, 3.0, 4.0)
        assert vector + other_point == Vector3(3.0, 5.0, 7.0)

    def test_sub(self):
        vector = Vector3(2.0, 3.0, 4.0)
        other = Vector3(1.0, 2.0, 3.0)

        assert vector - other == Vector3(1.0, 1.0, 1.0)

        other_point = Point(1.0, 2.0, 3.0)
        assert vector - other_point == Vector3(1.0, 1.0, 1.0)

    def test_mul(self):
        vector = Vector3(2.0, 3.0, 4.0)
        other = Vector3(1.0, 2.0, 3.0)

        assert vector * other == Vector3(2.0, 6.0, 12.0)

        other_point = Point(1.0, 2.0, 3.0)
        assert vector * other_point == Vector3(2.0, 6.0, 12.0)

    def test_rmul(self):
        vector = Vector3(2.0, 3.0, 4.0)

        assert 10 * vector == Vector3(20.0, 30.0, 40.0)
        assert 5.0 * vector == Vector3(10.0, 15.0, 20.0)
        new_vector = Euler(0.0, 0.0, 50.0).to_quaternion() * vector
        assert new_vector == Vector3(2.717056618096013, 2.3701483780684827, 4.0)

        vector = Vector3(1.0, 0.0, 0.0)
        new_vector = Euler(0.0, 0.0, math.pi / 2.0).to_quaternion() * vector
        assert new_vector == Vector3(0.0, 1.0, 0.0)

    def test_truediv(self):
        vector = Vector3(2.0, 3.0, 4.0)

        assert vector / 2.0 == Vector3(1.0, 1.5, 2.0)
        assert vector / 2 == Vector3(1.0, 1.5, 2.0)

    def test_iadd(self):
        vector = Vector3(2.0, 3.0, 4.0)

        point = Point(1.0, 2.0, 3.0)

        vector += point
        assert vector == Vector3(3.0, 5.0, 7.0)

    def test_isub(self):
        vector = Vector3(2.0, 3.0, 4.0)

        point = Point(1.0, 2.0, 3.0)

        vector -= point
        assert vector == Vector3(1.0, 1.0, 1.0)

    def test_imul(self):
        vector = Vector3(2.0, 3.0, 4.0)
        other = Point(1.0, 2.0, 3.0)

        vector *= other
        assert vector == Vector3(2.0, 6.0, 12.0)

        vector = Vector3(2.0, 3.0, 4.0)
        other = Vector3(1.0, 2.0, 3.0)

        vector *= other
        assert vector == Vector3(2.0, 6.0, 12.0)

        vector = Vector3(2.0, 3.0, 4.0)
        other = 2.0

        vector *= other
        assert vector == Vector3(4.0, 6.0, 8.0)

        vector = Vector3(2.0, 3.0, 4.0)
        other = 2

        vector *= other
        assert vector == Vector3(4.0, 6.0, 8.0)

    def test_idiv(self):
        vector = Vector3(2.0, 3.0, 4.0)
        other = 2.0

        vector /= other
        assert vector == Vector3(1.0, 1.5, 2.0)

        vector = Vector3(2.0, 3.0, 4.0)
        other = 2

        vector /= other
        assert vector == Vector3(1.0, 1.5, 2.0)

    def test_neg(self):
        vector = Vector3(2.0, 3.0, 4.0)
        assert -vector == Vector3(-2.0, -3.0, -4.0)

    def test_iter(self):
        vector = Vector3(2.0, 3.0, 4.0)
        for idx, component in enumerate(vector):
            if idx == 0:
                assert component == 2.0
            elif idx == 1:
                assert component == 3.0
            elif idx == 2:
                assert component == 4.0

    def test_eq(self):
        vector = Vector3(2.0, 3.0, 4.0)
        other = Vector3(2.0, 3.0, 4.0)
        assert vector == other

        other = Vector3(2.0, 3.0, 5.0)
        assert vector != other

    def test_getitem(self):
        vector = Vector3(2.0, 3.0, 4.0)
        assert vector[0] == 2.0
        assert vector[1] == 3.0
        assert vector[2] == 4.0

    def test_setitem(self):
        vector = Vector3()
        assert vector[0] == 0.0 and vector[1] == 0.0 and vector[2] == 0.0

        vector[0] = 2.0
        vector[1] = 3.0
        vector[2] = 4.0

        assert vector[0] == 2.0
        assert vector[1] == 3.0
        assert vector[2] == 4.0

    def test_distance(self):
        vector = Vector3(2.0, 3.0, 4.0)
        other = Vector3(3.0, 4.0, 5.0)

        assert vector.distance(other) == math.sqrt(3.0)

    def test_cross(self):
        vector = Vector3(2.0, 3.0, 4.0)
        other = Vector3(3.0, 4.0, 5.0)

        cross_vec = vector.cross(other)
        assert cross_vec == Vector3(-1.0, 2.0, -1.0)

    def test_dot(self):
        vector = Vector3(2.0, 3.0, 4.0)
        other = Vector3(3.0, 4.0, 5.0)

        assert vector.dot(other) == 38.0

    def test_norm(self):
        vector = Vector3(2.0, 3.0, 4.0)
        normalized_vector = vector.norm()
        assert normalized_vector == Vector3(0.3713906763541037, 0.5570860145311556, 0.7427813527082074)
        assert normalized_vector is not vector

    def test_normilize(self):
        vector = Vector3(2.0, 3.0, 4.0)
        normalized_vector = vector.normalize()
        assert normalized_vector == Vector3(0.3713906763541037, 0.5570860145311556, 0.7427813527082074)
        assert normalized_vector is vector
