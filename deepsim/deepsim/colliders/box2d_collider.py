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
"""A class for box 2D collider."""
from typing import Optional, List, Union

from deepsim.behaviours.transform import Transform
from deepsim.core.vector3 import Vector3
from deepsim.core.point import Point
from deepsim.core.pose import Pose
from deepsim.core.plane import Plane
from deepsim.core.ray import Ray
from deepsim.colliders.hit import Hit
from deepsim.colliders.abs_collider import Abstract2DCollider
from deepsim.gazebo.constants import GazeboWorld

from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry


class Box2DCollider(Abstract2DCollider):
    """
    Box2DCollider class
    """
    def __init__(self, length: float, width: float, *,
                 pose: Optional[Pose] = None,
                 pose_offset: Optional[Pose] = None,
                 transform: Optional[Transform] = None,
                 enable_on_init: bool = True) -> None:
        """
        Initialize Box2DCollider class
        - By default, Box2DCollider assumes its normal is always +z axis, and
          length lies on x-axis and width lies on y-axis.

        Args:
            length (float): length of the box
            width (float): width of the box
            pose: (Optional[Pose]): pose of the box collider.
            pose_offset (Optional[Pose]): pose offset.
            transform (Optional[Transform]): transform to be tracked by the collider.
            enable_on_init: (bool): the flag whether to enable the collider on initialize.
        """
        super(Box2DCollider, self).__init__(transform=transform,
                                            pose=pose,
                                            pose_offset=pose_offset,
                                            enable_on_init=enable_on_init)
        self._length = length
        self._width = width
        half_length = 0.5 * self._length
        half_width = 0.5 * self._width

        self._local_vertices = [Vector3.from_list([+half_length, +half_width, 0.0]),
                                Vector3.from_list([+half_length, -half_width, 0.0]),
                                Vector3.from_list([-half_length, -half_width, 0.0]),
                                Vector3.from_list([-half_length, +half_width, 0.0])]

    @property
    def length(self) -> float:
        """
        Returns the length of the box2d collider

        Returns:
            float: the length of the box.
        """
        return self._length

    @property
    def width(self) -> float:
        """
        Returns the width of the box2d collider

        Returns:
            float: the width of the box.
        """
        return self._width

    @property
    def points(self) -> List[Point]:
        """
        Returns the 4 points in Vector3 format representing the box.

        Returns:
            List[Point]: 4 points in Vector3 format representing the box.
        """
        pose = self.adjusted_pose
        return [pose.position + p.rotate(pose.orientation)
                for p in self._local_vertices]

    def to_shapely(self) -> BaseGeometry:
        """
        Returns Shapely Geometry representing the box collier.

        Returns:
            BaseGeometry: Shapely Geometry representing the box collier.
        """
        return Polygon([p.buffer[0:2] for p in self.points])

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
            if self.intersects(point_on_plane):
                hit = Hit(obj=self, ray=ray, entry=plane_hit.entry, exit=plane_hit.entry,
                          entry_normal=local_up,
                          exit_normal=-local_up)
        return hit

    def __repr__(self):
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Box2DCollider(%s)" % (str(self.points))
