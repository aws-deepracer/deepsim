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

import math

from deepsim.math import math as deepsim_math
from deepsim.math.quaternion import Quaternion
from deepsim.math.euler import Euler
from deepsim.math.vector3 import Vector3


class MathTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_euler_to_quaternion(self):
        buffer = deepsim_math.euler_to_quaternion(1.0, 2.0, 3.0)
        quaternion = Quaternion(buffer=buffer)
        expected_quaternion = Quaternion(-0.7182870182434113,
                                         0.31062245106570396,
                                         0.44443511344300074,
                                         0.4359528440735657)

        assert quaternion == expected_quaternion

    def test_quaternion_to_euler(self):
        quaternion = Quaternion(-0.7182870182434113,
                                0.31062245106570396,
                                0.44443511344300074,
                                0.4359528440735657)
        buffer = deepsim_math.quaternion_to_euler(*quaternion.to_list())
        euler = Euler(buffer=buffer)

        expected_euler = Euler(-2.141592653589793,
                               1.1415926535897936,
                               -0.14159265358979317)
        assert euler == expected_euler

    def test_project_to_2d(self):
        point_on_plane = Vector3(10.0, 5.0, 5.0)
        plane_center = Vector3(10.0, 0.0, 10.0)
        plane_width = 30.0
        plane_height = 15.0
        plane_quaternion = Euler(10.0, 20.0, 15.0).to_quaternion()
        space_in_2d = deepsim_math.project_to_2d(point_on_plane=point_on_plane,
                                                 plane_center=plane_center,
                                                 plane_width=plane_width,
                                                 plane_height=plane_height,
                                                 plane_quaternion=plane_quaternion)

        assert math.isclose(space_in_2d[0], 0.41058919533656374)
        assert math.isclose(space_in_2d[1], 0.31032871509751736)

    def test_lerp(self):
        current = 1.0
        target = 2.0
        fraction = 0.5
        assert deepsim_math.lerp(a=current, b=target, t=fraction) == 1.5
        assert deepsim_math.lerp(a=current, b=target, t=3.0) == target
        assert deepsim_math.lerp(a=current, b=target, t=-3.0) == current

    def test_lerp_angle_rad(self):
        current = math.pi
        target = math.pi * 2.0
        fraction = 0.5
        assert deepsim_math.lerp_angle_rad(a=current,
                                           b=target,
                                           t=fraction) == math.pi + math.pi / 2.0
        assert deepsim_math.lerp_angle_rad(a=current,
                                           b=target,
                                           t=3.0) == target
        assert deepsim_math.lerp_angle_rad(a=current,
                                           b=target,
                                           t=-3.0) == current

        current = math.pi * 3.0
        target = math.pi / 2.0
        fraction = 0.5
        assert deepsim_math.lerp_angle_rad(a=current,
                                           b=target,
                                           t=fraction) == 2.0 * math.pi + 3.0 * math.pi / 4.0

        current = math.pi / 2.0
        target = math.pi * 3.0
        fraction = 0.5
        assert deepsim_math.lerp_angle_rad(a=current,
                                           b=target,
                                           t=fraction) == 3.0 * math.pi / 4.0

    def test_cross(self):
        v1 = Vector3(1.0, 2.0, 3.0)
        v2 = Vector3(2.0, 3.0, 4.0)

        assert deepsim_math.cross(v1, v2) == v1.cross(v2)
        assert deepsim_math.cross(v1, v2) == Vector3(-1.0, 2.0, -1.0)

    def test_dot(self):
        v1 = Vector3(1.0, 2.0, 3.0)
        v2 = Vector3(2.0, 3.0, 4.0)

        assert deepsim_math.dot(v1, v2) == 20.0

    def test_magnitude(self):
        v1 = Vector3(1.0, 2.0, 3.0)
        assert deepsim_math.magnitude(v1) == math.sqrt(14.0)

        q1 = Quaternion(1.0, 2.0, 3.0, 4.0)
        assert deepsim_math.magnitude(q1) == math.sqrt(30.0)

    def test_sqr_magnitude(self):
        v1 = Vector3(1.0, 2.0, 3.0)
        assert deepsim_math.sqr_magnitude(v1) == 14.0

        q1 = Quaternion(1.0, 2.0, 3.0, 4.0)
        assert deepsim_math.sqr_magnitude(q1) == 30.0

    def test_unit(self):
        v1 = Vector3(1.0, 2.0, 3.0)
        assert deepsim_math.unit(v1) == Vector3(0.2672612419124244, 0.5345224838248488, 0.8017837257372732)
        q1 = Quaternion(1.0, 2.0, 3.0, 4.0)
        assert deepsim_math.unit(q1) == Quaternion(0.18257418583505536,
                                                   0.3651483716701107,
                                                   0.5477225575051661,
                                                   0.7302967433402214)

    def test_distance(self):
        v1 = Vector3(1.0, 2.0, 3.0)
        v2 = Vector3(2.0, 3.0, 4.0)
        assert deepsim_math.distance(v1, v2) == math.sqrt(3.0)








