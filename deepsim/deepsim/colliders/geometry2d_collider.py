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
"""A class for geometry 2D collider."""
from typing import Optional, Union

from deepsim.behaviours.transform import Transform
from deepsim.core.pose import Pose
from deepsim.core.plane import Plane
from deepsim.core.ray import Ray
from deepsim.colliders.hit import Hit
from deepsim.colliders.abs_collider import Abstract2DCollider
from deepsim.gazebo.constants import GazeboWorld

from shapely.geometry.base import BaseGeometry
from shapely import affinity


class Geometry2DCollider(Abstract2DCollider):
    """
    Geometry2DCollider class
    """
    def __init__(self, geometry: BaseGeometry, *,
                 pose: Optional[Pose] = None,
                 pose_offset: Optional[Pose] = None,
                 transform: Optional[Transform] = None,
                 enable_on_init: bool = True) -> None:
        """
        Initialize Geometry2DCollider class
        - By default, Mesh2DCollider assumes its normal is always +z axis.

        Args:
            geometry (BaseGeometryBaseGeometry): 2D collider geometry.
            pose: (Optional[Pose]): pose of the circle collider.
            pose_offset (Optional[Pose]): pose offset.
            transform (Optional[Transform]): transform to be tracked by the collider.
            enable_on_init: (bool): the flag whether to enable the collider on initialize.
        """
        super(Geometry2DCollider, self).__init__(transform=transform,
                                                 pose=pose,
                                                 pose_offset=pose_offset,
                                                 enable_on_init=enable_on_init)
        self._geometry = geometry

    @property
    def geometry(self) -> BaseGeometry:
        """
        Returns the geometry of the mesh collider.

        Returns:
            float: the geometry of the mesh collider.
        """
        return self._geometry

    def to_shapely(self) -> BaseGeometry:
        """
        Returns Shapely Geometry representing the circle collier.

        Returns:
            BaseGeometry: Shapely Geometry representing the circle collier.
        """
        pose = self.adjusted_pose
        position = pose.position
        yaw = pose.orientation.to_euler().yaw
        geometry = affinity.translate(self._geometry,
                                      position.x,
                                      position.y,
                                      position.z)
        return affinity.rotate(geometry, yaw, use_radians=True)

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
        return "Geometry2DCollider(adjusted_pose=%s)" % (repr(self.adjusted_pose))
