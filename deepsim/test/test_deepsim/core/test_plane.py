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

from deepsim.core.plane import Plane
from deepsim.core.point import Point
from deepsim.core.vector3 import Vector3
from deepsim.core.ray import Ray

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class PlaneTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        points = [Point(0.0, 1.0, 2.0),
                  Point(1.0, 2.0, 2.0),
                  Point(1.0, 1.0, 2.0)]
        point = Vector3(0.0, 2.0, 3.0)
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(points=points)
        assert plane.normal == Vector3(0.0, 0.0, -1.0)
        assert plane.distance == -2.0

        plane2 = Plane(normal=normal,
                       point=point)

        assert plane2.normal == Vector3(0.7071067811865476, 0.0, 0.7071067811865476)
        assert plane2.distance == 2.121320343559643

        plane3 = Plane(normal=normal,
                       distance=10.0)
        assert plane3.normal == normal.norm()
        assert plane3.distance == 10.0

        with self.assertRaises(ValueError):
            _ = Plane(distance=5.0)

        with self.assertRaises(ValueError):
            _ = Plane(point=point)

        with self.assertRaises(ValueError):
            _ = Plane(normal=normal)

    def test_setters(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)
        assert plane.normal == normal.norm()
        assert plane.distance == 10.0

        new_normal = Vector3(4.0, 0.0, 4.0)
        plane.normal = new_normal
        plane.distance = 5.0
        assert plane.normal == new_normal.norm()
        assert plane.distance == 5.0

    def test_flip_plane(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)

        flipped_plane = Plane.flip_plane(plane)
        assert flipped_plane.normal == -normal.norm()
        assert flipped_plane.distance == -10.0

    def test_flip(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)

        flipped_plane = plane.flip()
        assert flipped_plane.normal == -normal.norm()
        assert flipped_plane.distance == -10.0

    def test_flip_inplace(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)

        plane.flip_inplace()
        assert plane.normal == -normal.norm()
        assert plane.distance == -10.0

    def test_closest_point_on_plane(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)
        expected_closest_point = Vector3(2.071067811865479, 5.0, 12.071067811865479)

        vpoint = Vector3(10.0, 5.0, 20.0)
        closest_vpoint = plane.closest_point_on_plane(vpoint)

        assert closest_vpoint == expected_closest_point

        point = Point(10.0, 5.0, 20.0)
        closest_point = plane.closest_point_on_plane(point)
        assert closest_point == expected_closest_point

    def test_distance_to(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)

        point = Vector3(10.0, 5.0, 20.0)
        distance = plane.distance_to(point)

        expected_distance = 11.213203435596427
        assert distance == expected_distance

        normal = Vector3(0.0, 0.0, 1.0)

        plane = Plane(normal=normal,
                      distance=2.0)

        point = Vector3(0.0, 0.0, 20.0)
        distance = plane.distance_to(point)
        assert distance == 18.0

    def test_same_side(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)

        point = Vector3(20.0, 0.0, 20.0)
        point2 = Vector3(30.0, 0.0, 30.0)
        assert plane.same_side(point, point2)

        point = Vector3(20.0, 0.0, 20.0)
        point2 = Vector3(30.0, 0.0, -30.0)
        assert not plane.same_side(point, point2)

    def test_which_side(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)

        point = Vector3(20.0, 0.0, 20.0)
        assert plane.which_side(point) > 0
        point = Vector3(-20.0, 0.0, -20.0)
        assert plane.which_side(point) < 0

        normal = Vector3(0.0, 0.0, 3.0)
        plane = Plane(normal=normal,
                      distance=10.0)
        point = Vector3(0.0, 0.0, 10.0)
        assert plane.which_side(point) == 0.0

    def test_raycast(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)

        ray = Ray(origin=Vector3(20.0, 0.0, 20.0),
                  direction=Vector3(0.0, 0.0, -1.0))
        hit = plane.raycast(ray)
        assert hit is not None

        ray = Ray(origin=Vector3(20.0, 0.0, 20.0),
                  direction=Vector3(0.0, 0.0, 1.0))
        hit = plane.raycast(ray)
        assert hit is None

    def test_copy(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)
        plane_copy = plane.copy()

        assert plane_copy == plane
        assert plane_copy.normal == plane.normal
        assert plane_copy.normal is not plane.normal
        assert plane_copy.distance == plane.distance

    def test_eq(self):
        normal = Vector3(3.0, 0.0, 3.0)

        plane = Plane(normal=normal,
                      distance=10.0)
        plane2 = Plane(normal=normal,
                       distance=10.0)

        assert plane2 == plane
        assert plane2.normal == plane.normal
        assert plane2.normal is not plane.normal
        assert plane2.distance == plane.distance




