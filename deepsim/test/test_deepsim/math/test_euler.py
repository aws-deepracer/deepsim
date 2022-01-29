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

from deepsim.math.euler import Euler
import numpy as np


class EulerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        euler = Euler()
        assert euler.roll == 0.0 and euler.pitch == 0.0 and euler.yaw == 0.0

        euler2 = Euler(0.5, 0.6, 0.7)
        assert euler2.roll == 0.5 and euler2.pitch == 0.6 and euler2.yaw == 0.7

        # initialize with buffer
        euler3 = Euler(buffer=np.array([0.7, 0.6, 0.5]))
        assert euler3.roll == 0.7 and euler3.pitch == 0.6 and euler3.yaw == 0.5

        # buffer takes precedent.
        euler4 = Euler(0.5, 0.6, 0.7,
                       buffer=np.array([0.7, 0.6, 0.5]))
        assert euler4.roll == 0.7 and euler4.pitch == 0.6 and euler4.yaw == 0.5

        # buffer with smaller size
        with self.assertRaises(ValueError):
            _ = Euler(buffer=np.array([0.8, 0.7]))

        # ignore buffer element larger size
        euler5 = Euler(buffer=np.array([0.7, 0.6, 0.5, 0.4]))
        assert euler5.roll == 0.7 and euler5.pitch == 0.6 and euler5.yaw == 0.5

    def test_identity(self):
        assert Euler.identity() == Euler(0.0, 0.0, 0.0)

    def test_setter(self):
        euler = Euler()
        assert euler.roll == 0.0 and euler.pitch == 0.0 and euler.yaw == 0.0

        euler.roll = 0.5
        euler.pitch = 0.6
        euler.yaw = 0.7
        assert euler.roll == 0.5 and euler.pitch == 0.6 and euler.yaw == 0.7

    def test_to_list(self):
        euler = Euler(0.5, 0.6, 0.7)
        euler_list = euler.to_list()
        expected_list = [0.5, 0.6, 0.7]
        assert euler_list == expected_list

    def test_to_numpy(self):
        euler = Euler(0.5, 0.6, 0.7)
        euler_np = euler.to_numpy()
        expected_np_array = np.array([0.5, 0.6, 0.7])
        assert np.all(euler_np == expected_np_array)

    def test_to_quaternion(self):
        from deepsim.math.quaternion import Quaternion
        euler = Euler(0.5, 0.6, 0.7)
        quat = euler.to_quaternion()
        expected_quat = Quaternion(0.1238415000204839,
                                   0.3500188726294279,
                                   0.24871879769645677,
                                   0.8945887498444911)
        assert quat == expected_quat

    def test_from_list(self):
        euler_list = [0.5, 0.6, 0.7]

        euler = Euler.from_list(euler_list)
        assert euler == Euler(0.5, 0.6, 0.7)

    def test_from_numpy(self):
        np_array = np.array([0.5, 0.6, 0.7])

        euler = Euler.from_numpy(np_array)
        assert euler == Euler(0.5, 0.6, 0.7)

    def test_from_quaternion(self):
        from deepsim.math.quaternion import Quaternion
        quat = Quaternion(0.1238415000204839,
                          0.3500188726294279,
                          0.24871879769645677,
                          0.8945887498444911)

        euler = Euler.from_quaternion(quat)
        expected_euler = Euler(0.5, 0.6, 0.7)
        assert euler == expected_euler

        # tuple
        euler2 = Euler.from_quaternion((0.1238415000204839,
                                        0.3500188726294279,
                                        0.24871879769645677,
                                        0.8945887498444911))
        assert euler2 == expected_euler

        # list
        euler3 = Euler.from_quaternion([0.1238415000204839,
                                        0.3500188726294279,
                                        0.24871879769645677,
                                        0.8945887498444911])
        assert euler3 == expected_euler

        # np.array
        euler4 = Euler.from_quaternion(np.array([0.1238415000204839,
                                                 0.3500188726294279,
                                                 0.24871879769645677,
                                                 0.8945887498444911]))
        assert euler4 == expected_euler

    def test_copy(self):
        euler = Euler(0.5, 0.6, 0.7)
        euler_copy = euler.copy()
        assert euler == euler_copy
        assert euler is not euler_copy

    def test_iter(self):
        euler = Euler(0.5, 0.6, 0.7)
        for idx, component in enumerate(euler):
            if idx == 0:
                assert component == 0.5
            elif idx == 1:
                assert component == 0.6
            elif idx == 2:
                assert component == 0.7
            elif idx == 3:
                assert component == 0.8

    def test_eq(self):
        euler = Euler(0.5, 0.6, 0.7)
        euler2 = Euler(0.5, 0.6, 0.7)

        assert euler == euler2

        euler3 = Euler(0.8, 0.6, 0.7)

        assert euler != euler3

    def test_getitem(self):
        euler = Euler(0.5, 0.6, 0.7)
        assert euler[0] == 0.5
        assert euler[1] == 0.6
        assert euler[2] == 0.7

    def test_setitem(self):
        euler = Euler(0.5, 0.6, 0.7)
        euler[0] = 0.8
        euler[1] = 0.7
        euler[2] = 0.6
        assert euler == Euler(0.8, 0.7, 0.6)
