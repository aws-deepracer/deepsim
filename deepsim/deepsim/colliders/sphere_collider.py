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
"""A class for sphere collider."""
import math
from typing import Optional, Union

from deepsim.behaviours.transform import Transform
from deepsim.core.vector3 import Vector3
from deepsim.core.point import Point
from deepsim.core.pose import Pose
from deepsim.core.ray import Ray
from deepsim.colliders.hit import Hit
from deepsim.colliders.abs_collider import AbstractCollider, Abstract3DCollider


class SphereCollider(Abstract3DCollider):
    """
    SphereCollider class
    """
    def __init__(self, radius: float, *,
                 pose: Optional[Pose] = None,
                 pose_offset: Optional[Pose] = None,
                 transform: Optional[Transform] = None,
                 enable_on_init: bool = True) -> None:
        """
        Initialize SphereCollider class

        Args:
            radius (float): the radius of sphere collider.
            pose: (Optional[Pose]): pose of the sphere collider.
            pose_offset (Optional[Pose]): pose offset. (orientation offset will be ignored)
            transform (Optional[Transform]): transform to be tracked by the collider.
            enable_on_init: (bool): the flag whether to enable the collider on initialize.
        """
        super(SphereCollider, self).__init__(transform=transform,
                                             pose_offset=pose_offset,
                                             enable_on_init=enable_on_init)
        self._radius = radius
        self._pose = pose.copy() if pose else Pose()

    @property
    def radius(self) -> float:
        """
        Returns the radius of the sphere collider.

        Returns:
            float: the radius of the sphere.
        """
        return self._radius

    @property
    def center(self) -> Vector3:
        """
        Returns the center of the sphere collider.
        - offset is also incorporated.

        Returns:
            Vector3: the center of the sphere.
        """
        return self.adjusted_pose.position.to_vector()

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
        Preserve last transform position prior to detach.
        """
        transform = self.transform
        if transform:
            self._pose = transform.state.pose

    def _intersects(self, target: Union[AbstractCollider, Vector3, Point]) -> bool:
        """
        Check if the collider intersects with target.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether intersecting

        Returns:
            bool: True if intersects otherwise False.
        """
        from deepsim.colliders.box_collider import BoxCollider
        if isinstance(target, Vector3) or isinstance(target, Point):
            return (self.center - target).sqr_magnitude <= self.radius ** 2
        if isinstance(target, SphereCollider):
            distance_squared = (self.center - target.center).sqr_magnitude
            return distance_squared <= (self.radius + target.radius) ** 2
        if isinstance(target, BoxCollider):
            return target.intersects(self)
        return NotImplemented

    def _contains(self, target: Union[AbstractCollider, Vector3, Point]) -> bool:
        """
        Check if the collider contains the target.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether collider contains.

        Returns:
            bool: True if collider contains target otherwise False.
        """
        from deepsim.colliders.box_collider import BoxCollider
        if isinstance(target, Vector3) or isinstance(target, Point):
            return (self.center - target).sqr_magnitude < self.radius ** 2
        if isinstance(target, SphereCollider):
            if self.radius <= target.radius:
                # If this sphere's radius is smaller than or equal to target sphere
                # then this sphere cannot contain target sphere.
                return False
            distance_squared = (self.center - target.center).sqr_magnitude
            # distance + target.radius < self.radius means this sphere contains target sphere.
            # Optimized to avoid expensive square root operation:
            return distance_squared < (self.radius - target.radius) ** 2
        if isinstance(target, BoxCollider):
            # Get all points of box collider.
            points = target.points
            for point in points:
                # if any of the points are outside of sphere then
                # sphere doesn't contain this box collier.
                if not self.contains(point):
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
        hit = None
        q = self.center - ray.origin
        dir_dot_q = ray.direction.dot(q)
        square_diff = q.dot(q) - self.radius * self.radius
        discrim = dir_dot_q * dir_dot_q - square_diff
        if discrim >= 0:
            root = math.sqrt(discrim)
            t0 = (dir_dot_q - root)
            t1 = (dir_dot_q + root)
            t0_normal = (ray.get_point(t0) - self.center).norm()
            t1_normal = (ray.get_point(t1) - self.center).norm()
            if t0 < t1:
                hit = Hit(obj=self, ray=ray, entry=t0, exit=t1,
                          entry_normal=t0_normal, exit_normal=t1_normal)
            else:
                hit = Hit(obj=self, ray=ray, entry=t1, exit=t0,
                          entry_normal=t1_normal, exit_normal=t0_normal)
        return hit

    def __repr__(self):
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "SphereCollider(center=%s, radius=%.3f)" % (repr(self.center), self.radius)
