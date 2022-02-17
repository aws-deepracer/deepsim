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

from deepsim.colliders.abs_collider import AbstractCollider, Abstract3DCollider, ColliderType
from deepsim.core.pose import Pose
from deepsim.behaviours.transform import Transform
from deepsim.core.vector3 import Vector3
from deepsim.core.point import Point
from deepsim.core.euler import Euler
from deepsim.core.ray import Ray
from deepsim.colliders.hit import Hit


class Dummy3DCollider(Abstract3DCollider):
    def __init__(self,
                 transform: Optional[Transform] = None, *,
                 pose_offset: Optional[Pose] = None,
                 enable_on_init: bool = True):
        self.mock = MagicMock()
        super(Dummy3DCollider, self).__init__(transform=transform,
                                              pose_offset=pose_offset,
                                              enable_on_init=enable_on_init)

    def on_attach(self) -> None:
        self.mock.on_attach()

    def on_detach(self):
        self.mock.on_detach()

    def _intersects(self, target: Union['AbstractCollider', Vector3, Point]) -> bool:
        return self.mock.intersects(target)

    def _contains(self, target: Union['AbstractCollider', Vector3, Point]) -> bool:
        return self.mock.contains(target)

    def raycast(self, ray: Ray) -> Union[Hit, None]:
        return self.mock.raycast(ray)


class AbstractColliderTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        collider = Dummy3DCollider()
        assert collider.is_enabled
        assert collider.pose_offset == Pose()
        assert collider.type == ColliderType.COLLIDER_3D

    def test_initialize_custom(self):
        pose_offset = Pose(Point(1.0, 2.0, 3.0), Euler(math.pi, math.pi / 2.0, 0.0).to_quaternion())
        transform_mock = MagicMock()
        collider = Dummy3DCollider(pose_offset=pose_offset,
                                   transform=transform_mock)
        assert collider.type == ColliderType.COLLIDER_3D
        assert collider.pose_offset == pose_offset
        assert collider.transform == transform_mock

    def test_initialize_disable_on_init(self):
        collider = Dummy3DCollider(enable_on_init=False)
        assert not collider.is_enabled

    def test_enable(self):
        collider = Dummy3DCollider(enable_on_init=False)
        collider.enable()
        assert collider.is_enabled

    def test_disable(self):
        collider = Dummy3DCollider(enable_on_init=False)
        collider.enable()
        assert collider.is_enabled

        collider.disable()
        assert not collider.is_enabled

    def test_attach(self):
        collider = Dummy3DCollider()
        transform_mock = MagicMock()
        collider.attach(transform=transform_mock)

        assert collider.transform == transform_mock
        collider.mock.on_attach.assert_called_once()

    def test_detach(self):
        collider = Dummy3DCollider()
        transform_mock = MagicMock()
        collider.attach(transform=transform_mock)

        assert collider.transform == transform_mock
        collider.mock.on_attach.assert_called_once()

        collider.detach()
        assert collider.transform is None
        collider.mock.on_detach.assert_called_once()

    def test_validate(self):
        collider_3d = Dummy3DCollider()

        vector_target = Vector3()
        point_target = Point()
        collider_3d_target = Dummy3DCollider()

        collider_3d.validate(vector_target)
        collider_3d.validate(point_target)
        collider_3d.validate(collider_3d_target)

    def test_pose_offset(self):
        collider_3d = Dummy3DCollider()

        pose_offset_mock = MagicMock()

        collider_3d.pose_offset = pose_offset_mock

        assert collider_3d.pose_offset == pose_offset_mock.copy.return_value.copy.return_value

    def test_intersect(self):
        collider_3d = Dummy3DCollider()

        vector_target = Vector3()
        point_target = Point()
        collider_3d_target = Dummy3DCollider()

        collider_3d.intersects(vector_target)
        collider_3d.intersects(point_target)
        collider_3d.intersects(collider_3d_target)

        collider_3d.mock.intersects.has_calls(
            call(vector_target),
            call(point_target),
            call(collider_3d_target)
        )
        assert collider_3d.mock.intersects.call_count == 3

    def test_contains(self):
        collider_3d = Dummy3DCollider()

        vector_target = Vector3()
        point_target = Point()
        collider_3d_target = Dummy3DCollider()

        collider_3d.contains(vector_target)
        collider_3d.contains(point_target)
        collider_3d.contains(collider_3d_target)

        collider_3d.mock.contains.has_calls(
            call(vector_target),
            call(point_target),
            call(collider_3d_target)
        )
        assert collider_3d.mock.contains.call_count == 3

    def test_contains_ops(self):
        collider_3d = Dummy3DCollider()

        vector_target = Vector3()
        point_target = Point()
        collider_3d_target = Dummy3DCollider()

        _ = vector_target in collider_3d
        _ = point_target in collider_3d
        _ = collider_3d_target in collider_3d

        collider_3d.mock.contains.has_calls(
            call(vector_target),
            call(point_target),
            call(collider_3d_target)
        )
        assert collider_3d.mock.contains.call_count == 3

    def test_raycast(self):
        collider_3d = Dummy3DCollider()

        origin_mock = MagicMock()
        direction_mock = MagicMock()
        ray = Ray(origin=origin_mock,
                  direction=direction_mock)

        collider_3d.raycast(ray)

        collider_3d.mock.raycast.assert_called_once_with(ray)
