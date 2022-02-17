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
"""A class for box collider."""
from typing import Optional, List, Union
import math

import numpy as np

from deepsim.behaviours.transform import Transform
from deepsim.core.vector3 import Vector3
from deepsim.core.point import Point
from deepsim.core.pose import Pose
from deepsim.core.ray import Ray
from deepsim.colliders.hit import Hit
from deepsim.colliders.abs_collider import AbstractCollider, Abstract3DCollider
from deepsim.gazebo.constants import GazeboWorld


class BoxCollider(Abstract3DCollider):
    """
    BoxCollider class
    """
    def __init__(self, length: float, width: float, height: float, *,
                 pose: Optional[Pose] = None,
                 pose_offset: Optional[Pose] = None,
                 transform: Optional[Transform] = None,
                 enable_on_init: bool = True) -> None:
        """
        Initialize BoxCollider class

        Args:
            length (float): length of the box (x-axis)
            width (float): width of the box (y-axis)
            height (float): height of the box (z-axis)
            pose: (Optional[Pose]): pose of the box collider.
            pose_offset (Optional[Pose]): pose offset.
            transform (Optional[Transform]): transform to be tracked by the collider.
            enable_on_init: (bool): the flag whether to enable the collider on initialize.
        """
        super(BoxCollider, self).__init__(transform=transform,
                                          pose_offset=pose_offset,
                                          enable_on_init=enable_on_init)
        self._length = length
        self._width = width
        self._height = height
        half_length = 0.5 * self._length
        half_width = 0.5 * self._width
        half_height = 0.5 * self._height
        self._pose = pose.copy() if pose else Pose()

        self._local_vertices = [Vector3.from_list([-half_length, -half_width, -half_height]),
                                Vector3.from_list([+half_length, -half_width, -half_height]),
                                Vector3.from_list([+half_length, +half_width, -half_height]),
                                Vector3.from_list([-half_length, +half_width, -half_height]),
                                Vector3.from_list([-half_length, -half_width, +half_height]),
                                Vector3.from_list([+half_length, -half_width, +half_height]),
                                Vector3.from_list([+half_length, +half_width, +half_height]),
                                Vector3.from_list([-half_length, +half_width, +half_height])]

    @property
    def length(self) -> float:
        """
        Returns the length of the box collider

        Returns:
            float: the length of the box.
        """
        return self._length

    @property
    def width(self) -> float:
        """
        Returns the width of the box collider

        Returns:
            float: the width of the box.
        """
        return self._width

    @property
    def height(self) -> float:
        """
        Returns the height of the box collider

        Returns:
            float: the height of the box.
        """
        return self._height

    @property
    def x_axis(self) -> Vector3:
        """
        Returns the x-axis of the box collider

        Returns:
            Vector3: the x-axis of the box.
        """
        return GazeboWorld.FORWARD.vector.rotate(self.adjusted_pose.orientation)

    @property
    def y_axis(self) -> Vector3:
        """
        Returns the y-axis of the box collider

        Returns:
            Vector3: the y-axis of the box.
        """
        return GazeboWorld.LEFT.vector.rotate(self.adjusted_pose.orientation)

    @property
    def z_axis(self) -> Vector3:
        """
        Returns the z-axis of the box collider

        Returns:
            Vector3: the z-axis of the box.
        """
        return GazeboWorld.UP.vector.rotate(self.adjusted_pose.orientation)

    @property
    def pose(self) -> Pose:
        """
        Returns the copy of pose.
        - If tracking transform then it will return transform pose, otherwise will return the copy of internal pose.

        Returns:
            Pose: transform pose if tracking transform otherwise the copy of internal pose.
        """
        transform = self.transform
        if transform:
            return transform.state.pose
        return self._pose.copy()

    @pose.setter
    def pose(self, value: Pose) -> None:
        """
        Set pose with given pose.
        - This won't have any impact during tracking the transform.

        Args:
            value (Pose): new pose
        """
        self._pose = value.copy()

    @property
    def adjusted_pose(self) -> Pose:
        """
        Returns the adjusted pose (pose + pose_offset).

        Returns:
            Pose: the adjusted pose (pose + pose_offset).
        """
        return self.pose + self._pose_offset

    def on_detach(self) -> None:
        """
        Preserve last transform pose prior to detach.
        """
        transform = self.transform
        if transform:
            self._pose = transform.state.pose

    @property
    def points(self) -> List[Point]:
        """
        Returns the 8 points in Vector3 format representing the box.

        Returns:
            List[Point]: 8 points in Vector3 format representing the box.
        """
        pose = self.adjusted_pose
        return [pose.position + p.rotate(pose.orientation)
                for p in self._local_vertices]

    def _b2b_intersects(self, target: 'BoxCollider') -> bool:
        """
        Checks whether given box collider intersects with this box collider.

        Args:
            target (BoxCollider): target box collider to check the intersection with this box collider.

        Returns:
            bool: True if this box collider intersects with given box collider, otherwise False.
        """
        axes = [self.x_axis, self.y_axis, self.z_axis,
                target.x_axis, target.y_axis, target.z_axis,
                self.x_axis.cross(target.x_axis).norm(),
                self.x_axis.cross(target.y_axis).norm(),
                self.x_axis.cross(target.z_axis).norm(),
                self.y_axis.cross(target.x_axis).norm(),
                self.y_axis.cross(target.y_axis).norm(),
                self.y_axis.cross(target.z_axis).norm(),
                self.z_axis.cross(target.x_axis).norm(),
                self.z_axis.cross(target.y_axis).norm(),
                self.z_axis.cross(target.z_axis).norm(),
                ]

        for axis in axes:
            if not self.is_overlap_on_axis(self, target, axis):
                return False

        return True

    @staticmethod
    def is_overlap_on_axis(box1: 'BoxCollider', box2: 'BoxCollider', axis: Vector3) -> bool:
        """
        Returns True if box1 overlap with box2 on given axis.

        Args:
            box1 (BoxCollider): box1
            box2 (BoxCollider): box2
            axis (Vector3): axis to check overlap of given box1 and box2.

        Returns:
            bool: True if box1 and box2 overlaps on given axis, otherwise False.
        """
        box1_points = box1.points
        box2_points = box2.points

        box1_projected_points = []
        box2_projected_points = []
        for idx in range(len(box1_points)):
            box1_projected_points.append(BoxCollider.get_signed_projection_value(box1_points[idx], axis))
            box2_projected_points.append(BoxCollider.get_signed_projection_value(box2_points[idx], axis))

        box1_min = min(box1_projected_points)
        box1_max = max(box1_projected_points)
        box2_min = min(box2_projected_points)
        box2_max = max(box2_projected_points)

        return box2_min <= box1_max and box1_min <= box2_max

    @staticmethod
    def get_signed_projection_value(point: Union[Vector3, Point], axis: Vector3) -> float:
        """
        Return signed projected value of given point to given axis.

        Args:
            point (Union[Vector3, Point]): point to project
            axis (Vector3): axis

        Returns:
            float: signed projected value from point to axis.
        """
        if isinstance(point, Point):
            point = point.to_vector()
        projected_point = Vector3.project(point, axis)
        return projected_point.magnitude * math.copysign(1.0, projected_point.dot(axis))

    def _intersects_point(self, target: Union[Vector3, Point]) -> bool:
        """
        Returns true if this box collider intersects target point, otherwise false.

        Args:
            target Union[Vector3, Point]: point to check whether it intersects with this box collider.

        Returns:
            True if this box collider intersects given point, otherwise False.
        """
        direction = target - self.adjusted_pose.position
        axes = [self.x_axis, self.y_axis, self.z_axis]
        sizes = [self.length / 2.0, self.width / 2.0, self.height / 2.0]
        for idx in range(3):
            axis = axes[idx]

            distance = direction.dot(axis)

            if distance > sizes[idx]:
                return False
            if distance < -sizes[idx]:
                return False
        return True

    def _contains_point(self, target: Union[Vector3, Point]) -> bool:
        """
        Returns true if this box collider contains target point, otherwise false.

        Args:
            target Union[Vector3, Point]: point to check whether it is inside of this box collider.

        Returns:
            True if this box collider contains given point, otherwise False.
        """
        direction = target - self.adjusted_pose.position
        axes = [self.x_axis, self.y_axis, self.z_axis]
        sizes = [self.length / 2.0, self.width / 2.0, self.height / 2.0]
        for idx in range(3):
            axis = axes[idx]
            distance = direction.dot(axis)

            if distance >= sizes[idx]:
                return False
            if distance <= -sizes[idx]:
                return False
        return True

    def _closest_point(self, target: Union[Vector3, Point]) -> Vector3:
        """
        Find the closest point in box collider to target.

        Args:
            target Union[Vector3, Point]: the target to find the closest point in this box collider.

        Returns:
            Vector3: the closest point on box collider.
        """
        result = self.adjusted_pose.position
        direction = target - result

        axes = [self.x_axis, self.y_axis, self.z_axis]
        sizes = [self.length / 2.0, self.width / 2.0, self.height / 2.0]

        for idx in range(3):
            axis = axes[idx]
            distance = direction.dot(axis)

            if distance > sizes[idx]:
                distance = sizes[idx]
            if distance < -sizes[idx]:
                distance = - sizes[idx]

            result = result + (axis * distance)
        return result

    def _intersects(self, target: Union[AbstractCollider, Vector3, Point]) -> bool:
        """
        Check if the collider intersects with target.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether intersecting

        Returns:
            bool: True if intersects otherwise False.
        """
        from deepsim.colliders.sphere_collider import SphereCollider
        if isinstance(target, Vector3) or isinstance(target, Point):
            return self._intersects_point(target)
        if isinstance(target, BoxCollider):
            return self._b2b_intersects(target)
        if isinstance(target, SphereCollider):
            closest_point = self._closest_point(target.center)
            sqr_dist = (target.center - closest_point).sqr_magnitude
            sqr_radius = target.radius * target.radius
            return sqr_dist <= sqr_radius
        return NotImplemented

    def _contains(self, target: Union[AbstractCollider, Vector3, Point]) -> bool:
        """
        Check if the collider contains with target.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether collider contains.

        Returns:
            bool: True if collider contains target otherwise False.
        """
        from deepsim.colliders.sphere_collider import SphereCollider
        if isinstance(target, Vector3) or isinstance(target, Point):
            return self._contains_point(target)
        if isinstance(target, BoxCollider):
            points = target.points
            for point in points:
                # if any of the points are outside of box collider then
                # this box collider doesn't contain target box collier.
                if not self.contains(point):
                    return False
            return True
        if isinstance(target, SphereCollider):
            # Translate sphere, so box collider is centered at origin.
            target_on_origin = target.center - self.adjusted_pose.position
            # Rotate sphere with inverse quaternion of box's orientation,
            # to project the sphere in box's local
            target_on_box_local = target_on_origin.rotate(self.adjusted_pose.orientation.inverse())
            if abs(target_on_box_local.x) + target.radius >= self.length / 2.0:
                return False
            if abs(target_on_box_local.y) + target.radius >= self.width / 2.0:
                return False
            if abs(target_on_box_local.z) + target.radius >= self.height / 2.0:
                return False
            return True

        return NotImplemented

    def raycast(self, ray: Ray) -> Union[Hit, None]:
        """
        Returns the distance along the ray, where it intersects the plane.
        - If there is no intersection then returns None.

        Args:
            ray (Ray): ray to test intersection.

        Returns:
            Union[Hit, None]: Hit object if intersects. Otherwise, None.
        """
        eps = 0.00001

        p = self.adjusted_pose.position - ray.origin

        e = Vector3(self.x_axis.dot(p),
                    self.y_axis.dot(p),
                    self.z_axis.dot(p))

        f = Vector3(self.x_axis.dot(ray.direction),
                    self.y_axis.dot(ray.direction),
                    self.z_axis.dot(ray.direction))

        t = [0.0] * 6
        sizes = [self.length / 2.0, self.width / 2.0, self.height / 2.0]

        for idx in range(3):
            if np.isclose(f[idx], 0.0):
                if -e[idx] - sizes[idx] > 0.0 or -e[idx] + sizes[idx] < 0:
                    return None
                f[idx] = eps
            t[idx * 2 + 0] = e[idx] + sizes[idx] / f[idx]
            t[idx * 2 + 1] = e[idx] - sizes[idx] / f[idx]

        tmin = max(max(min(t[0], t[1]), min(t[2], t[3])), min(t[4], t[5]))
        tmax = min(min(max(t[0], t[1]), max(t[2], t[3])), max(t[4], t[5]))

        if tmax < 0.0:
            return None

        if tmin > tmax:
            return None

        normals = [self.x_axis,
                   -self.x_axis,
                   self.y_axis,
                   -self.y_axis,
                   self.z_axis,
                   -self.z_axis]

        tmin_normal = None
        tmax_normal = None
        for idx, t_val in enumerate(t):
            if t_val == tmin:
                tmin_normal = normals[idx]
            if t_val == tmax:
                tmax_normal = normals[idx]

        if tmin >= 0.0:
            hit = Hit(obj=self, ray=ray,
                      entry=tmin, exit=tmax,
                      entry_normal=tmin_normal, exit_normal=tmax_normal)
        else:
            hit = Hit(obj=self, ray=ray,
                      entry=tmax, exit=tmin,
                      entry_normal=tmax_normal, exit_normal=tmin_normal)
        return hit

    def __repr__(self):
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "BoxCollider(%s)" % (str(self.points))
