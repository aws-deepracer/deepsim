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

from deepsim.math.color import Color
from std_msgs.msg import ColorRGBA
import numpy as np


class ColorTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        # default initialization
        color_def = Color()
        assert color_def.r == 0.0 and color_def.g == 0.0 and color_def.b == 0.0 and color_def.a == 1.0

        # initialize with  rgba
        color = Color(0.5, 0.6, 0.7, 0.8)
        assert color.r == 0.5 and color.g == 0.6 and color.b == 0.7 and color.a == 0.8

        # initialize with buffer
        color2 = Color(buffer=np.array([0.8, 0.7, 0.6, 0.5]))
        assert color2.r == 0.8 and color2.g == 0.7 and color2.b == 0.6 and color2.a == 0.5

        # buffer takes precedent.
        color3 = Color(0.5, 0.6, 0.7, 0.8,
                       buffer=np.array([0.8, 0.7, 0.6, 0.5]))
        assert color3.r == 0.8 and color3.g == 0.7 and color3.b == 0.6 and color3.a == 0.5

        # buffer with smaller size
        with self.assertRaises(ValueError):
            _ = Color(buffer=np.array([0.8, 0.7, 0.6]))

        color4 = Color(buffer=np.array([0.8, 0.7, 0.6, 0.5, 0.4]))
        assert color4.r == 0.8 and color4.g == 0.7 and color4.b == 0.6 and color4.a == 0.5

    def test_setter(self):
        color = Color()
        assert color.r == 0.0 and color.g == 0.0 and color.b == 0.0 and color.a == 1.0

        color.r = 0.5
        color.g = 0.6
        color.b = 0.7
        color.a = 0.8
        assert color.r == 0.5 and color.g == 0.6 and color.b == 0.7 and color.a == 0.8

    def test_to_ros(self):
        color = Color(0.5, 0.6, 0.7, 0.8)
        ros_color = color.to_ros()
        expected_ros_color = ColorRGBA()
        expected_ros_color.r = 0.5
        expected_ros_color.g = 0.6
        expected_ros_color.b = 0.7
        expected_ros_color.a = 0.8

        assert ros_color == expected_ros_color

    def test_to_list(self):
        color = Color(0.5, 0.6, 0.7, 0.8)
        color_list = color.to_list()
        expected_list = [0.5, 0.6, 0.7, 0.8]
        assert color_list == expected_list

    def test_to_numpy(self):
        color = Color(0.5, 0.6, 0.7, 0.8)
        color_np = color.to_numpy()
        expected_np_array = np.array([0.5, 0.6, 0.7, 0.8])
        assert np.all(color_np == expected_np_array)

    def test_from_ros(self):
        ros_color = ColorRGBA()
        ros_color.r = 0.5
        ros_color.g = 0.6
        ros_color.b = 0.7
        ros_color.a = 0.8

        color = Color.from_ros(ros_color)

        assert color == Color(0.5, 0.6, 0.7, 0.8)

    def test_from_list(self):
        color_list = [0.5, 0.6, 0.7, 0.8]

        color = Color.from_list(color_list)
        assert color == Color(0.5, 0.6, 0.7, 0.8)

    def test_from_numpy(self):
        np_array = np.array([0.5, 0.6, 0.7, 0.8])

        color = Color.from_numpy(np_array)
        assert color == Color(0.5, 0.6, 0.7, 0.8)

    def test_copy(self):
        color = Color(0.5, 0.6, 0.7, 0.8)
        color_copy = color.copy()
        assert color == color_copy
        assert color is not color_copy

    def test_iter(self):
        color = Color(0.5, 0.6, 0.7, 0.8)
        for idx, component in enumerate(color):
            if idx == 0:
                assert component == 0.5
            elif idx == 1:
                assert component == 0.6
            elif idx == 2:
                assert component == 0.7
            elif idx == 3:
                assert component == 0.8

    def test_eq(self):
        color = Color(0.5, 0.6, 0.7, 0.8)
        color2 = Color(0.5, 0.6, 0.7, 0.8)

        assert color == color2

        color3 = Color(0.8, 0.6, 0.7, 0.8)

        assert color != color3

    def test_getitem(self):
        color = Color(0.5, 0.6, 0.7, 0.8)
        assert color[0] == 0.5
        assert color[1] == 0.6
        assert color[2] == 0.7
        assert color[3] == 0.8

    def test_setitem(self):
        color = Color(0.5, 0.6, 0.7, 0.8)
        color[0] = 0.8
        color[1] = 0.7
        color[2] = 0.6
        color[3] = 0.5
        assert color == Color(0.8, 0.7, 0.6, 0.5)
