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
from typing import Any, Callable, Optional, Iterable, Union
from unittest import TestCase
from unittest.mock import patch, MagicMock, call
import inspect

import math

from deepsim.colliders.abs_collider import Abstract2DCollider, ColliderType
from deepsim.core.pose import Pose
from deepsim.behaviours.transform import Transform
from deepsim.core.vector3 import Vector3
from deepsim.core.point import Point
from deepsim.core.euler import Euler
from deepsim.core.ray import Ray
from deepsim.colliders.hit import Hit

from shapely.geometry.base import BaseGeometry


class Dummy2DCollider(Abstract2DCollider):
    def __init__(self,
                 transform: Optional[Transform] = None, *,
                 pose: Optional[Pose] = None,
                 pose_offset: Optional[Pose] = None,
                 enable_on_init: bool = True):
        self.mock = MagicMock()
        super(Dummy2DCollider, self).__init__(transform=transform,
                                              pose=pose,
                                              pose_offset=pose_offset,
                                              enable_on_init=enable_on_init)

    def to_shapely(self) -> BaseGeometry:
        return self.mock.to_shapely()

    def raycast(self, ray: Ray) -> Union[Hit, None]:
        return self.mock.raycast(ray)


class AbstractColliderTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        collider = Dummy2DCollider()
        assert collider.is_enabled
        assert collider.pose == Pose()
        assert collider.pose_offset == Pose()
        assert collider.adjusted_pose == Pose() + Pose()
        assert collider.type == ColliderType.COLLIDER_2D

    def test_initialize_custom_without_transform(self):
        pose = Pose(Point(3.0, 2.0, 1.0), Euler(math.pi, 0.0, math.pi / 2.0).to_quaternion())
        pose_offset = Pose(Point(1.0, 2.0, 3.0), Euler(math.pi, math.pi / 2.0, 0.0).to_quaternion())
        collider = Dummy2DCollider(pose=pose,
                                   pose_offset=pose_offset)

        assert collider.pose_offset == pose_offset
        assert collider.pose == pose
        assert collider.adjusted_pose == pose + pose_offset

    def test_initialize_custom_with_transform(self):
        pose = Pose(Point(3.0, 2.0, 1.0), Euler(math.pi, 0.0, math.pi / 2.0).to_quaternion())
        transform_mock = MagicMock()
        transform_mock.state.pose = Pose(Point(1.0, 1.0, 1.0), Euler(math.pi, 0.0, math.pi / 3.0).to_quaternion())
        pose_offset = Pose(Point(1.0, 2.0, 3.0), Euler(math.pi, math.pi / 2.0, 0.0).to_quaternion())
        collider = Dummy2DCollider(pose=pose,
                                   pose_offset=pose_offset,
                                   transform=transform_mock)

        assert collider.pose_offset == pose_offset
        assert collider.pose == transform_mock.state.pose
        assert collider.pose != pose
        assert collider.adjusted_pose == transform_mock.state.pose + pose_offset
        assert collider.adjusted_pose != pose + pose_offset

    def test_initialize_disable_on_init(self):
        collider = Dummy2DCollider(enable_on_init=False)
        assert not collider.is_enabled

    def test_detach(self):
        pose = Pose(Point(3.0, 2.0, 1.0), Euler(math.pi, 0.0, math.pi / 2.0).to_quaternion())
        transform_mock = MagicMock()
        transform_mock.state.pose = Pose(Point(1.0, 1.0, 1.0), Euler(math.pi, 0.0, math.pi / 3.0).to_quaternion())
        pose_offset = Pose(Point(1.0, 2.0, 3.0), Euler(math.pi, math.pi / 2.0, 0.0).to_quaternion())
        collider = Dummy2DCollider(pose=pose,
                                   pose_offset=pose_offset,
                                   transform=transform_mock)

        collider.detach()
        assert collider.transform is None
        assert collider.pose == transform_mock.state.pose
        assert collider.adjusted_pose == transform_mock.state.pose + pose_offset

    def test_pose(self):
        pose = Pose(Point(3.0, 2.0, 1.0), Euler(math.pi, 0.0, math.pi / 2.0).to_quaternion())
        new_pose = Pose(Point(1.0, 1.0, 1.0), Euler(math.pi, 0.0, math.pi / 3.0).to_quaternion())
        pose_offset = Pose(Point(1.0, 2.0, 3.0), Euler(math.pi, math.pi / 2.0, 0.0).to_quaternion())
        collider = Dummy2DCollider(pose=pose,
                                   pose_offset=pose_offset)

        assert collider.pose == pose
        assert collider.adjusted_pose == pose + pose_offset
        collider.pose = new_pose
        assert collider.pose == new_pose
        assert collider.adjusted_pose == new_pose + pose_offset

    def test_intersect(self):
        collider_2d = Dummy2DCollider()

        vector_target = Vector3()
        point_target = Point()
        collider_2d_target = Dummy2DCollider()

        collider_2d.intersects(vector_target)
        collider_2d.intersects(point_target)
        collider_2d.intersects(collider_2d_target)

        collider_2d.mock.to_shapely.return_value.intersects.has_calls(
            call(vector_target.to_shapely()),
            call(point_target.to_shapely()),
            call(collider_2d_target.to_shapely())
        )

    def test_contains(self):
        collider_2d = Dummy2DCollider()

        vector_target = Vector3()
        point_target = Point()
        collider_2d_target = Dummy2DCollider()

        collider_2d.contains(vector_target)
        collider_2d.contains(point_target)
        collider_2d.contains(collider_2d_target)

        collider_2d.mock.contains.has_calls(
            call(vector_target.to_shapely()),
            call(point_target.to_shapely()),
            call(collider_2d_target.to_shapely())
        )

    def test_contains_ops(self):
        collider_2d = Dummy2DCollider()

        vector_target = Vector3()
        point_target = Point()
        collider_2d_target = Dummy2DCollider()

        _ = vector_target in collider_2d
        _ = point_target in collider_2d
        _ = collider_2d_target in collider_2d

        collider_2d.mock.contains.has_calls(
            call(vector_target.to_shapely()),
            call(point_target.to_shapely()),
            call(collider_2d_target.to_shapely())
        )

    def test_raycast(self):
        collider_2d = Dummy2DCollider()

        origin_mock = MagicMock()
        direction_mock = MagicMock()
        ray = Ray(origin=origin_mock,
                  direction=direction_mock)

        collider_2d.raycast(ray)

        collider_2d.mock.raycast.assert_called_once_with(ray)
