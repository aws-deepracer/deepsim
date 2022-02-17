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

from deepsim.core.quaternion import Quaternion
from deepsim.core.vector3 import Vector3
from deepsim.core.euler import Euler
import numpy as np

from geometry_msgs.msg import Quaternion as ROSQuaternion


class QuaternionTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        quaternion = Quaternion()
        assert quaternion.x == 0.0 and quaternion.y == 0.0 and quaternion.z == 0.0 and quaternion.w == 1.0

        quaternion2 = Quaternion(0.5, 0.6, 0.7, 0.8)
        assert quaternion2.x == 0.5 and quaternion2.y == 0.6 and quaternion2.z == 0.7 and quaternion2.w == 0.8

        # initialize with buffer
        quaternion3 = Quaternion(buffer=np.array([0.7, 0.6, 0.5, 0.4]))
        assert quaternion3.x == 0.7 and quaternion3.y == 0.6 and quaternion3.z == 0.5 and quaternion3.w == 0.4

        # buffer takes precedent.
        quaternion4 = Quaternion(0.5, 0.6, 0.7,
                                 buffer=np.array([0.7, 0.6, 0.5, 0.4]))
        assert quaternion4.x == 0.7 and quaternion4.y == 0.6 and quaternion4.z == 0.5 and quaternion4.w == 0.4

        # buffer with smaller size
        with self.assertRaises(ValueError):
            _ = Quaternion(buffer=np.array([0.8, 0.7, 0.6]))

        # ignore buffer element larger size
        quaternion5 = Quaternion(buffer=np.array([0.7, 0.6, 0.5, 0.4, 0.3]))
        assert quaternion5.x == 0.7 and quaternion5.y == 0.6 and quaternion5.z == 0.5 and quaternion5.w == 0.4

    def test_identity(self):
        assert Quaternion.identity() == Quaternion(0.0, 0.0, 0.0, 1.0)

    def test_lerp(self):
        q1 = Euler(math.pi / 2.0).to_quaternion()
        q2 = Euler(math.pi).to_quaternion()
        new_q = Quaternion.lerp(q1, q2, 0.5)
        expected_q = Euler(3.0 * math.pi / 4.0).to_quaternion()
        assert new_q == expected_q

    def test_slerp(self):
        q1 = Euler(math.pi / 2.0).to_quaternion()
        q2 = Euler(math.pi).to_quaternion()
        new_q = Quaternion.slerp(q1, q2, 0.5)
        expected_q = Euler(3.0 * math.pi / 4.0).to_quaternion()
        assert new_q == expected_q

    def test_look_rotation(self):
        forward = Vector3(0.0, 1.0, 0.0)
        up = Vector3(0.0, 0.0, 1.0)
        rot = Quaternion.look_rotation(forward, up)
        expected_rot = Euler(yaw=math.pi/2.0).to_quaternion()
        assert rot == expected_rot

        forward = Vector3(0.0, 0.0, 1.0)
        up = Vector3(-1.0, 0.0, 0.0)
        rot = Quaternion.look_rotation(forward, up)
        expected_rot = Euler(pitch=-math.pi / 2.0).to_quaternion()
        assert rot == expected_rot

    def test_setter(self):
        quaternion = Quaternion()
        assert quaternion.x == 0.0 and quaternion.y == 0.0 and quaternion.z == 0.0 and quaternion.w == 1.0

        quaternion.x = 0.5
        quaternion.y = 0.6
        quaternion.z = 0.7
        quaternion.w = 0.8
        assert quaternion.x == 0.5 and quaternion.y == 0.6 and quaternion.z == 0.7 and quaternion.w == 0.8

    def test_sqr_magnitude(self):
        quaternion = Quaternion(1.0, 2.0, 3.0, 4.0)
        assert quaternion.sqr_magnitude == 30.0

    def test_magnitude(self):
        quaternion = Quaternion(1.0, 2.0, 3.0, 4.0)
        assert quaternion.magnitude == math.sqrt(30.0)

    def test_to_ros(self):
        ros_quaternion = ROSQuaternion()
        ros_quaternion.x = 1.0
        ros_quaternion.y = 2.0
        ros_quaternion.z = 3.0
        ros_quaternion.w = 4.0

        quaternion = Quaternion(1.0, 2.0, 3.0, 4.0)
        assert quaternion.to_ros() == ros_quaternion

    def test_to_list(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion_list = quaternion.to_list()
        expected_list = [0.5, 0.6, 0.7, 0.8]
        assert quaternion_list == expected_list

    def test_to_numpy(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion_np = quaternion.to_numpy()
        expected_np_array = np.array([0.5, 0.6, 0.7, 0.8])
        assert np.all(quaternion_np == expected_np_array)

    def test_to_euler(self):
        from deepsim.core.euler import Euler
        quaternion = Quaternion(0.1238415000204839,
                                0.3500188726294279,
                                0.24871879769645677,
                                0.8945887498444911)
        euler = quaternion.to_euler()
        expected_euler = Euler(0.5, 0.6, 0.7)
        assert euler == expected_euler

    def test_inverse_inplace(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion.inverse_inplace()
        assert quaternion == Quaternion(-0.28735632183908044,
                                        -0.3448275862068965,
                                        -0.4022988505747126,
                                        0.4597701149425288)

    def test_inverse(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion_inverse = quaternion.inverse()
        assert quaternion_inverse == Quaternion(-0.28735632183908044,
                                                -0.3448275862068965,
                                                -0.4022988505747126,
                                                0.4597701149425288)

    def test_from_ros(self):
        ros_quaternion = ROSQuaternion()
        ros_quaternion.x = 1.0
        ros_quaternion.y = 2.0
        ros_quaternion.z = 3.0
        ros_quaternion.w = 4.0

        assert Quaternion.from_ros(ros_quaternion) == Quaternion(1.0, 2.0, 3.0, 4.0)

    def test_from_list(self):
        quaternion_list = [0.5, 0.6, 0.7, 0.8]

        quaternion = Quaternion.from_list(quaternion_list)
        assert quaternion == Quaternion(0.5, 0.6, 0.7, 0.8)

    def test_from_numpy(self):
        np_array = np.array([0.5, 0.6, 0.7, 0.8])

        quaternion = Quaternion.from_numpy(np_array)
        assert quaternion == Quaternion(0.5, 0.6, 0.7, 0.8)

    def test_from_euler(self):
        from deepsim.core.euler import Euler
        euler = Euler(0.5, 0.6, 0.7)

        quaternion = Quaternion.from_euler(euler)
        expected_quaternion = Quaternion(0.1238415000204839,
                                         0.3500188726294279,
                                         0.24871879769645677,
                                         0.8945887498444911)
        assert quaternion == expected_quaternion

        # tuple
        quaternion2 = Quaternion.from_euler((0.5, 0.6, 0.7))
        assert quaternion2 == expected_quaternion

        # list
        quaternion3 = Quaternion.from_euler([0.5, 0.6, 0.7])
        assert quaternion3 == expected_quaternion

        # np.array
        quaternion4 = Quaternion.from_euler(np.array([0.5, 0.6, 0.7]))
        assert quaternion4 == expected_quaternion

    def test_copy(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion_copy = quaternion.copy()
        assert quaternion == quaternion_copy
        assert quaternion is not quaternion_copy

    def test_add(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion2 = Quaternion(0.6, 0.7, 0.8, 0.9)
        assert quaternion + quaternion2 == Quaternion(1.1, 1.3, 1.5, 1.7)

    def test_sub(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion2 = Quaternion(0.6, 0.7, 0.8, 0.9)
        assert quaternion2 - quaternion == Quaternion(0.1, 0.1, 0.1, 0.1)

    def test_mul(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        assert quaternion * 2.0 == Quaternion(1.0, 1.2, 1.4, 1.6)
        assert quaternion * 2 == Quaternion(1.0, 1.2, 1.4, 1.6)

    def test_rmul(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        assert 2.0 * quaternion == Quaternion(1.0, 1.2, 1.4, 1.6)
        assert 2 * quaternion == Quaternion(1.0, 1.2, 1.4, 1.6)

    def test_truediv(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        assert quaternion / 2.0 == Quaternion(0.25, 0.3, 0.35, 0.4)
        assert quaternion / 2 == Quaternion(0.25, 0.3, 0.35, 0.4)

    def test_iadd(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion2 = Quaternion(0.6, 0.7, 0.8, 0.9)
        quaternion += quaternion2
        assert quaternion == Quaternion(1.1, 1.3, 1.5, 1.7)

    def test_isub(self):
        quaternion = Quaternion(0.6, 0.7, 0.8, 0.9)
        quaternion2 = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion -= quaternion2
        assert quaternion == Quaternion(0.1, 0.1, 0.1, 0.1)

    def test_imul(self):
        quaternion = Quaternion(0.6, 0.7, 0.8, 0.9)
        quaternion *= 2.0
        assert quaternion == Quaternion(1.2, 1.4, 1.6, 1.8)

        quaternion = Quaternion(0.6, 0.7, 0.8, 0.9)
        quaternion *= 2
        assert quaternion == Quaternion(1.2, 1.4, 1.6, 1.8)

    def test_idiv(self):
        quaternion = Quaternion(0.6, 0.7, 0.8, 0.9)
        quaternion /= 2.0
        assert quaternion == Quaternion(0.3, 0.35, 0.4, 0.45)

        quaternion = Quaternion(0.6, 0.7, 0.8, 0.9)
        quaternion /= 2
        assert quaternion == Quaternion(0.3, 0.35, 0.4, 0.45)

    def test_neg(self):
        quaternion = Quaternion(0.6, 0.7, 0.8, 0.9)
        assert -quaternion == Quaternion(-0.6, -0.7, -0.8, -0.9)

    def test_iter(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        for idx, component in enumerate(quaternion):
            if idx == 0:
                assert component == 0.5
            elif idx == 1:
                assert component == 0.6
            elif idx == 2:
                assert component == 0.7
            elif idx == 3:
                assert component == 0.8

    def test_eq(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion2 = Quaternion(0.5, 0.6, 0.7, 0.8)

        assert quaternion == quaternion2

        quaternion3 = Quaternion(0.8, 0.6, 0.7, 0.8)

        assert quaternion != quaternion3

    def test_getitem(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        assert quaternion[0] == 0.5
        assert quaternion[1] == 0.6
        assert quaternion[2] == 0.7
        assert quaternion[3] == 0.8

    def test_setitem(self):
        quaternion = Quaternion(0.5, 0.6, 0.7, 0.8)
        quaternion[0] = 0.8
        quaternion[1] = 0.7
        quaternion[2] = 0.6
        quaternion[3] = 0.5
        assert quaternion == Quaternion(0.8, 0.7, 0.6, 0.5)

    def test_dot(self):
        quaternion = Quaternion(1.0, 2.0, 3.0, 4.0)
        quaternion2 = Quaternion(2.0, 3.0, 4.0, 5.0)
        assert quaternion.dot(quaternion2) == 40.0

    def test_norm(self):
        quaternion = Quaternion(1.0, 2.0, 3.0, 4.0)
        quat_norm = quaternion.norm()
        assert quat_norm == Quaternion(0.18257418583505536,
                                       0.3651483716701107,
                                       0.5477225575051661,
                                       0.7302967433402214)
        assert quat_norm is not quaternion

    def test_normalize(self):
        quaternion = Quaternion(1.0, 2.0, 3.0, 4.0)
        quat_norm = quaternion.normalize()
        assert quat_norm == Quaternion(0.18257418583505536,
                                       0.3651483716701107,
                                       0.5477225575051661,
                                       0.7302967433402214)
        assert quat_norm is quaternion
