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

from deepsim.core.pose import Pose
from deepsim.core.vector3 import Vector3
from deepsim.core.quaternion import Quaternion
from deepsim.core.point import Point
from deepsim.core.euler import Euler
from deepsim.core.ray import Ray

from deepsim.colliders.box2d_collider import Box2DCollider
from deepsim.colliders.circle2d_collider import Circle2DCollider
from deepsim.colliders.geometry2d_collider import Geometry2DCollider
from deepsim.colliders.sphere_collider import SphereCollider
from deepsim.colliders.abs_collider import ColliderType

from shapely.geometry.polygon import Polygon


class Geometry2DColliderTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)

        assert collider.geometry == polygon
        assert collider.is_enabled

    def test_adjusted_pose(self):
        pose = Pose(Point(1.0, 2.0, 3.0),
                    Euler(30.0).to_quaternion())
        pose_offset = Pose(Point(2.0, 3.0, 4.0),
                           Euler(0.0, 30.0).to_quaternion())

        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon,
                                      pose=pose,
                                      pose_offset=pose_offset)

        expected_adjusted_pose = pose + pose_offset
        assert expected_adjusted_pose == collider.adjusted_pose

    def test_enable(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon,
                                      enable_on_init=False)
        assert not collider.is_enabled
        collider.enable()
        assert collider.is_enabled

    def test_disable(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)

        assert collider.is_enabled
        collider.disable()
        assert not collider.is_enabled

    def test_type(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)
        assert collider.type == ColliderType.COLLIDER_2D

    def test_pose(self):
        pose = MagicMock()
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon,
                                      pose=pose)
        assert collider.pose == pose.copy.return_value.copy.return_value
        transform_mock = MagicMock()
        collider.attach(transform_mock)
        assert collider.pose == transform_mock.state.pose

    def test_pose_offset(self):
        pose_offset = MagicMock()
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon,
                                      pose_offset=pose_offset)
        assert collider.pose_offset == pose_offset.copy.return_value.copy.return_value

    def test_transform(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)
        assert collider.transform is None
        transform_mock = MagicMock()
        collider.attach(transform_mock)
        assert collider.transform == transform_mock
        collider.detach()
        assert collider.transform is None

    def test_raycast(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)
        ray = Ray(origin=Vector3(0.0, 0.0, -1.0),
                  direction=Vector3(0.0, 0.0, 1.0))
        hit = collider.raycast(ray)
        assert hit is not None

    def test_raycast_miss(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)
        ray = Ray(origin=Vector3(0.0, 0.0, 1.0),
                  direction=Vector3(0.0, 0.0, 1.0))
        hit = collider.raycast(ray)
        assert hit is None

    def test_contains(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)
        small_collider = Box2DCollider(length=1.0,
                                       width=1.0)
        assert collider.contains(small_collider)

        circle_collider = Circle2DCollider(1.0)
        assert collider.contains(circle_collider)

        polygon = Polygon([[-1, -1],
                           [-1, 1],
                           [1, 1],
                           [1, -1]])
        geom2d_collider = Geometry2DCollider(geometry=polygon)
        assert collider.contains(geom2d_collider)

        vector1 = Vector3(0.0, 0.0, 0.0)
        assert collider.contains(vector1)

        point1 = Point(0.0, 0.0, 0.0)
        assert collider.contains(point1)

    def test_not_contains(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)

        large_collider = Box2DCollider(length=1.5,
                                       width=4.0)
        assert not collider.contains(large_collider)

        circle_collider = Circle2DCollider(radius=2.0)
        assert not collider.contains(circle_collider)

        polygon = Polygon([[-1, -1],
                           [-1, 4],
                           [1, 1],
                           [1, -1]])
        geom2d_collider = Geometry2DCollider(geometry=polygon)
        assert not collider.contains(geom2d_collider)

        vector1 = Vector3(0.0, 3.0, 0.0)
        assert not collider.contains(vector1)

        point1 = Point(0.0, 3.0, 0.0)
        assert not collider.contains(point1)

    def test_contains_3d_collider(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)

        sphere_collider = SphereCollider(3.0)
        with self.assertRaises(ValueError):
            collider.contains(sphere_collider)

    def test_intersects(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)

        small_collider = Box2DCollider(length=4.0,
                                       width=1.0)
        assert collider.intersects(small_collider)

        circle_collider = Circle2DCollider(4.0)
        assert collider.intersects(circle_collider)

        polygon = Polygon([[-1, -1],
                           [-1, 4],
                           [1, 1],
                           [1, -1]])
        geom2d_collider = Geometry2DCollider(geometry=polygon)
        assert collider.intersects(geom2d_collider)

        vector1 = Vector3(1.0, 0.0, 0.0)
        assert collider.intersects(vector1)

        point1 = Point(1.5, 0.0, 0.0)
        assert collider.intersects(point1)

    def test_not_intersects(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)
        small_collider = Box2DCollider(length=1.5,
                                       width=1.0,
                                       pose=Pose(Point(5.0, 5.0, 0.0)))
        assert not collider.intersects(small_collider)

        circle_collider = Circle2DCollider(1.0,
                                           pose=Pose(Point(5.0, 5.0, 0.0)))
        assert not collider.intersects(circle_collider)

        polygon = Polygon([[3, 4],
                           [3, 8],
                           [4, 8],
                           [4, 4]])
        geom2d_collider = Geometry2DCollider(geometry=polygon)
        assert not collider.intersects(geom2d_collider)

        vector1 = Vector3(4.0, 0.0, 0.0)
        assert not collider.intersects(vector1)

        point1 = Point(3.0, 0.0, 0.0)
        assert not collider.intersects(point1)

    def test_intersects_3d_collider(self):
        polygon = Polygon([[-1.5, -1.5],
                           [-1.5, 1.5],
                           [1.5, 1.5],
                           [1.5, -1.5]])
        collider = Geometry2DCollider(geometry=polygon)

        sphere_collider = SphereCollider(3.0)
        with self.assertRaises(ValueError):
            collider.intersects(sphere_collider)
