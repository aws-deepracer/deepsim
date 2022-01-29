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

from deepsim.math.ray import Ray
from deepsim.math.vector3 import Vector3
from deepsim.math.point import Point


class RayTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        ray = Ray()
        assert ray.origin == Vector3()
        assert ray.direction == Vector3()

        origin = Vector3(1.0, 2.0, 3.0)
        direction = Vector3(2.0, 3.0, 4.0)
        ray = Ray(origin=origin, direction=direction)
        assert ray.origin == origin
        assert ray.origin is not origin
        assert ray.direction == direction.norm()

        origin = Point(1.0, 2.0, 3.0)
        ray = Ray(origin=origin, direction=direction)
        assert type(ray.origin) == Vector3
        assert ray.origin == origin
        assert ray.origin is not origin
        assert ray.direction == direction.norm()

    def test_setter(self):
        ray = Ray()
        assert ray.origin == Vector3()
        assert ray.direction == Vector3()

        origin = Vector3(1.0, 2.0, 3.0)
        direction = Vector3(2.0, 3.0, 4.0)

        ray.origin = origin
        ray.direction = direction

        assert ray.origin == origin
        assert ray.origin is not origin
        assert ray.direction == direction.norm()

    def test_get_point(self):
        origin = Vector3(0.0, 0.0, 3.0)
        direction = Vector3(1.0, 0.0, 0.0)
        ray = Ray(origin=origin,
                  direction=direction)
        point = ray.get_point(10.0)
        assert point == Vector3(10.0, 0.0, 3.0)

    def test_eq(self):
        origin = Vector3(0.0, 0.0, 3.0)
        direction = Vector3(1.0, 0.0, 0.0)
        ray = Ray(origin=origin,
                  direction=direction)
        ray2 = Ray(origin=origin,
                   direction=direction)
        assert ray == ray2
        assert ray.origin == ray2.origin
        assert ray.origin is not ray2.origin
        assert ray.direction == ray2.direction
        assert ray.direction is not ray2. direction

        direction = Vector3(1.0, 0.0, 1.0)
        ray2 = Ray(origin=origin,
                   direction=direction)
        assert ray != ray2
