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
"""A class for circle 2D collider."""
from typing import Optional, Union

from deepsim.behaviours.transform import Transform
from deepsim.math.pose import Pose
from deepsim.math.plane import Plane
from deepsim.math.ray import Ray
from deepsim.colliders.hit import Hit
from deepsim.colliders.abs_collider import Abstract2DCollider
from deepsim.gazebo.constants import GazeboWorld

from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.base import BaseGeometry


class Circle2DCollider(Abstract2DCollider):
    """
    Circle2DCollider class
    """
    def __init__(self, radius: float, *,
                 pose: Optional[Pose] = None,
                 pose_offset: Optional[Pose] = None,
                 transform: Optional[Transform] = None,
                 enable_on_init: bool = True) -> None:
        """
        Initialize Circle2DCollider class
        - By default, Circle2DCollider assumes its normal is always +z axis.

        Args:
            radius (float): the radius of the circle collider
            pose: (Optional[Pose]): pose of the circle collider.
            pose_offset (Optional[Pose]): pose offset.
            transform (Optional[Transform]): transform to be tracked by the collider.
            enable_on_init: (bool): the flag whether to enable the collider on initialize.
        """
        super(Circle2DCollider, self).__init__(transform=transform,
                                               pose=pose,
                                               pose_offset=pose_offset,
                                               enable_on_init=enable_on_init)
        self._radius = radius

    @property
    def radius(self) -> float:
        """
        Returns the radius of the circle collider.

        Returns:
            float: the radius of the circle.
        """
        return self._radius

    def to_shapely(self) -> BaseGeometry:
        """
        Returns Shapely Geometry representing the circle collier.

        Returns:
            BaseGeometry: Shapely Geometry representing the circle collier.
        """
        p = ShapelyPoint(self.adjusted_pose.position.buffer[0:2])
        return p.buffer(self.radius)

    def raycast(self, ray: Ray) -> Union[Hit, None]:
        """
        Returns the distance along the ray, where it intersects the collider.
        - If there is no intersection then returns None.

        Args:
            ray (Ray): ray to test intersection.

        Returns:
            Union[Hit, None]: Hit object if intersects. Otherwise, None.
        """
        hit = None
        collider_pose = self.adjusted_pose
        local_up = GazeboWorld.UP.vector.rotate(collider_pose.orientation)
        plane = Plane(normal=local_up,
                      point=collider_pose.position)
        plane_hit = plane.raycast(ray)
        if plane_hit:
            point_on_plane = ray.get_point(plane_hit.entry)
            if (point_on_plane - collider_pose.position).sqr_magnitude <= self.radius ** 2:
                # If distance between the center of circle and point on plane is smaller or equal to the circle's radius
                # then it's a hit. (Using squared distance and squared radius for better performance.)
                hit = Hit(obj=self, ray=ray, entry=plane_hit.entry, exit=plane_hit.entry,
                          entry_normal=local_up, exit_normal=-local_up)
        return hit

    def __repr__(self):
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Circle2DCollider(adjusted_pose=%s, radius=%.3f)" % (repr(self.adjusted_pose), self.radius)
