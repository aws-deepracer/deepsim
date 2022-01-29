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

from deepsim.math.pose import Pose
from deepsim.math.vector3 import Vector3
from deepsim.math.point import Point
from deepsim.math.euler import Euler
from deepsim.math.ray import Ray

from deepsim.colliders.box_collider import BoxCollider
from deepsim.colliders.sphere_collider import SphereCollider
from deepsim.colliders.circle2d_collider import Circle2DCollider
from deepsim.colliders.abs_collider import ColliderType

from shapely.geometry.polygon import Polygon


class BoxColliderTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)

        assert collider.width == width
        assert collider.length == length
        assert collider.height == height

        half_length = 0.5 * length
        half_width = 0.5 * width
        half_height = 0.5 * height
        local_vertices = [Vector3.from_list([-half_length, -half_width, -half_height]),
                          Vector3.from_list([+half_length, -half_width, -half_height]),
                          Vector3.from_list([+half_length, +half_width, -half_height]),
                          Vector3.from_list([-half_length, +half_width, -half_height]),
                          Vector3.from_list([-half_length, -half_width, +half_height]),
                          Vector3.from_list([+half_length, -half_width, +half_height]),
                          Vector3.from_list([+half_length, +half_width, +half_height]),
                          Vector3.from_list([-half_length, +half_width, +half_height])]
        pose = Pose()
        expected_points = [pose.position + p.rotate(pose.orientation)
                           for p in local_vertices]

        assert collider.points == expected_points
        assert collider.is_enabled

    def test_adjusted_pose(self):
        pose = Pose(Point(1.0, 2.0, 3.0),
                    Euler(30.0).to_quaternion())
        pose_offset = Pose(Point(2.0, 3.0, 4.0),
                           Euler(0.0, 30.0).to_quaternion())

        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height,
                               pose=pose,
                               pose_offset=pose_offset)

        assert collider.width == width
        assert collider.length == length
        assert collider.height == height

        half_length = 0.5 * length
        half_width = 0.5 * width
        half_height = 0.5 * height
        local_vertices = [Vector3.from_list([-half_length, -half_width, -half_height]),
                          Vector3.from_list([+half_length, -half_width, -half_height]),
                          Vector3.from_list([+half_length, +half_width, -half_height]),
                          Vector3.from_list([-half_length, +half_width, -half_height]),
                          Vector3.from_list([-half_length, -half_width, +half_height]),
                          Vector3.from_list([+half_length, -half_width, +half_height]),
                          Vector3.from_list([+half_length, +half_width, +half_height]),
                          Vector3.from_list([-half_length, +half_width, +half_height])]

        adjusted_pose = pose + pose_offset
        expected_points = [adjusted_pose.position + p.rotate(adjusted_pose.orientation)
                           for p in local_vertices]

        assert collider.points == expected_points

    def test_enable(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height,
                               enable_on_init=False)
        assert not collider.is_enabled
        collider.enable()
        assert collider.is_enabled

    def test_disable(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)
        assert collider.is_enabled
        collider.disable()
        assert not collider.is_enabled

    def test_type(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)
        assert collider.type == ColliderType.COLLIDER_3D

    def test_pose(self):
        pose = MagicMock()
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height,
                               pose=pose)
        assert collider.pose == pose.copy.return_value.copy.return_value
        transform_mock = MagicMock()
        collider.attach(transform_mock)
        assert collider.pose == transform_mock.state.pose

    def test_pose_offset(self):
        pose_offset = MagicMock()
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height,
                               pose_offset=pose_offset)
        assert collider.pose_offset == pose_offset.copy.return_value.copy.return_value

    def test_transform(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)
        assert collider.transform is None
        transform_mock = MagicMock()
        collider.attach(transform_mock)
        assert collider.transform == transform_mock
        collider.detach()
        assert collider.transform is None

    def test_raycast(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)
        ray = Ray(origin=Vector3(0.0, 0.0, -1.0),
                  direction=Vector3(0.0, 0.0, 1.0))
        hit = collider.raycast(ray)
        assert hit is not None

    def test_raycast_miss(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)
        ray = Ray(origin=Vector3(0.0, 0.0, 3.0),
                  direction=Vector3(0.0, 0.0, 1.0))
        hit = collider.raycast(ray)
        assert hit is None

    def test_contains(self):
        length = 3.0
        width = 3.0
        height = 3.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)
        small_collider = BoxCollider(length=1.5,
                                     width=1.0,
                                     height=1.0)
        assert collider.contains(small_collider)

        sphere_collider = SphereCollider(1.0)
        assert collider.contains(sphere_collider)

        vector1 = Vector3(0.0, 0.0, 0.0)
        assert collider.contains(vector1)

        point1 = Point(0.0, 0.0, 0.0)
        assert collider.contains(point1)

    def test_not_contains(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)

        large_collider = BoxCollider(length=1.5,
                                     width=2.5,
                                     height=1.5)
        assert not collider.contains(large_collider)

        sphere_collider = SphereCollider(2.5)
        assert not collider.contains(sphere_collider)

        vector1 = Vector3(0.0, 3.0, 0.0)
        assert not collider.contains(vector1)

        point1 = Point(0.0, 3.0, 0.0)
        assert not collider.contains(point1)

    def test_contains_2d_collider(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)

        circl2d_collider = Circle2DCollider(3.0)
        with self.assertRaises(ValueError):
            collider.contains(circl2d_collider)

    def test_intersects(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)
        small_collider = BoxCollider(length=3.0,
                                     width=1.0,
                                     height=2.0)
        assert collider.intersects(small_collider)

        sphere_collider = SphereCollider(4.0)
        assert collider.intersects(sphere_collider)

        vector1 = Vector3(1.0, 0.0, 0.0)
        assert collider.intersects(vector1)

        point1 = Point(1.5, 0.0, 0.0)
        assert collider.intersects(point1)

    def test_not_intersects(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)
        small_collider = BoxCollider(length=1.5,
                                     width=1.0,
                                     height=1.0,
                                     pose=Pose(Point(5.0, 5.0, 5.0)))
        assert not collider.intersects(small_collider)

        sphere_collider = SphereCollider(1.0,
                                         pose=Pose(Point(5.0, 5.0, 5.0)))
        assert not collider.intersects(sphere_collider)

        vector1 = Vector3(4.0, 0.0, 0.0)
        assert not collider.intersects(vector1)

        point1 = Point(3.0, 0.0, 0.0)
        assert not collider.intersects(point1)

    def test_intersects_2d_collider(self):
        length = 3.0
        width = 2.0
        height = 2.0

        collider = BoxCollider(length=length,
                               width=width,
                               height=height)

        circl2d_collider = Circle2DCollider(3.0)
        with self.assertRaises(ValueError):
            collider.intersects(circl2d_collider)
