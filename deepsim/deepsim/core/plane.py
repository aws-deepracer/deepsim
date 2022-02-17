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
"""A class for plane."""
from typing import Optional, Union, List, Tuple
import numpy as np
from deepsim.core.vector3 import Vector3
from deepsim.core.point import Point
from deepsim.core.ray import Ray
from deepsim.colliders.hit import Hit
from deepsim.colliders.ray_castable import RayCastable


class Plane(RayCastable):
    """
    Plane class
    """
    def __init__(self,
                 normal: Optional[Vector3] = None,
                 distance: Optional[float] = None,
                 point: Optional[Union[Vector3, Point]] = None,
                 points: Optional[Union[List[Vector3], List[Point],
                                        Tuple[Vector3, Vector3, Vector3],
                                        Tuple[Point, Point, Point]]] = None) -> None:
        """
        Instantiate the plane. Must have one of following combination to instantiate the plane:
        - points: containing 3 of Vector3 representing the plane.
        - normal and point: plane normal and single point on the plane
        - normal and distance: plane normal and distance from origin to directly instantiate the plane.

        Args:
            normal (Optional[Vector3]): plane normal
            distance (Optional[float]): the distance measured from the plane to the origin along the plane's normal
            point (Optional[Union[Vector3, Point]]): single point on the plane
            points (Optional[Union[List[Vector3], List[Point],
                                        Tuple[Vector3, Vector3, Vector3],
                                        Tuple[Point, Point, Point]]]): 3 points representing the plane.
        """
        if points:
            p0, p1, p2 = points[0], points[1], points[2]
            self._normal = (p1 - p0).cross(p2 - p0).norm()
            self._constant = -self._normal.norm().dot(p0)
        elif normal and point:
            self._normal = normal.norm()
            self._constant = -self._normal.norm().dot(point)
        elif normal and distance is not None:
            self._normal = normal.norm()
            self._constant = -distance
        else:
            err_msg = "Must provied either 3 points[List[Vector3]], (normal and point[Vector3]), or (normal and constant)."
            raise ValueError(err_msg)

    @property
    def normal(self) -> Vector3:
        """
        Returns the copy of plane normal.

        Returns:
            Vector3: the copy of plane normal.
        """
        return self._normal.copy()

    @normal.setter
    def normal(self, value: Vector3) -> None:
        """
        Set new plane normal.

        Args:
            value (Vector3): the new plane normal
        """
        self._normal = value.norm()

    @property
    def distance(self) -> float:
        """
        Returns the distance measured from the plane to the origin along the plane's normal.

        Returns:
            float: the distance measured from the plane to the origin along the plane's normal.
        """
        return -self._constant

    @distance.setter
    def distance(self, value: float) -> None:
        """
        Set the distance measured from the plane to the origin along the plane's normal.

        Args:
            value (float):the distance measured from the plane to the origin along the plane's normal.
        """
        self._constant = -value

    @staticmethod
    def _flip_plane(plane: 'Plane') -> Tuple[Vector3, float]:
        """
        Returns a new plane that face the opposite direction.

        Args:
            plane (Plane): the plane to flip.

        Returns:
            Tuple[Vector3, float]: normal and constant of flipped plane.
        """
        return -plane._normal, -plane._constant

    @staticmethod
    def flip_plane(plane: 'Plane') -> 'Plane':
        """
        Returns a new plane that face the opposite direction.

        Args:
            plane (Plane): the plane to flip.

        Returns:
            Plane: flipped plane.
        """
        normal, constant = Plane._flip_plane(plane=plane)
        return Plane(normal=normal, distance=-constant)

    def flip(self) -> 'Plane':
        """
        Returns a new plane that face the opposite direction.

        Returns:
            Plane: The plane facing opposite direction.
        """
        flipped_normal, flipped_constant = Plane._flip_plane(self)
        return Plane(normal=flipped_normal, distance=-flipped_constant)

    def flip_inplace(self) -> 'Plane':
        """
        Flip the current plane and return self.

        Returns:
            Plane: self after flip.
        """
        self._normal, self._constant = Plane._flip_plane(self)
        return self

    def closest_point_on_plane(self, point: Union[Vector3, Point]) -> Vector3:
        """
        Return the closest point on plane with respect to given point.

        Args:
            point (Union[Vector3, Point]): the point to find the closest point on the plane.

        Returns:
            Vector3: the closest point on the plane from given point.
        """
        distance = self.distance_to(point=point)
        return point - (self._normal * distance)

    def distance_to(self, point: Union[Vector3, Point]) -> float:
        """
        Returns signed distance.
        - The sign of the return value is positive if the point is on the positive side of the plane, negative if
        the point is on the negative side, and zero if the point is on the plane.

        Args:
            point (Union[Vector3, Point]): the point to find distance from the plane.

        Returns:
            float: the signed distance.
        """
        return self._normal.dot(point) + self._constant

    def same_side(self, p0: Union[Vector3, Point], p1: Union[Vector3, Point]) -> bool:
        """
        Check if given two points are on the same side of the plane.

        Args:
            p0 (Union[Vector3, Point]): point 0
            p1 (Union[Vector3, Point]): point 1

        Returns:
            bool: True if two points are on same side of the plane, otherwise False.
        """
        d0 = self.distance_to(p0)
        d1 = self.distance_to(p1)
        return (d0 > 0.0 and d1 > 0.0) or (d0 <= 0.0 and d1 <= 0.0)

    def which_side(self, point: Union[Vector3, Point]) -> float:
        """
        Returns +1 for the positive side, -1 for the negative side, and 0 for the point being on the plane.
        -  The "positive side" of the plane is the half space to which the plane normal points.
           The "negative side" is the other half space.

        Args:
            point (Union[Vector3, Point]): the point to find side with respect to the plane.

        Returns:
            float: +1 for the positive side, -1 for the negative side, and 0 for the point being on the plane
        """
        distance = self.distance_to(point)
        if distance < 0.0:
            return -1.0
        if distance > 0.0:
            return 1.0
        return 0.0

    def raycast(self, ray: Ray) -> Union[Hit, None]:
        """
        Returns the distance along the ray, where it intersects the plane.
        - If there is no intersection then returns None.

        Args:
            ray (Ray): ray to test intersection.

        Returns:
            Union[Hit, None]: Hit object if intersects. Otherwise, None.
        """
        plane_normal = self._normal
        ray_dir = ray.direction
        ray_origin = ray.origin

        denominator = plane_normal.dot(ray_dir)
        hit = None
        if not np.isclose(denominator, 0.0):
            t = (-ray_origin.dot(plane_normal) - self._constant) / denominator
            if t >= 0.0:
                hit = Hit(obj=self, ray=ray, entry=t, exit=t,
                          entry_normal=plane_normal, exit_normal=-plane_normal)
        return hit

    def copy(self) -> 'Plane':
        """
        Returns a copy.

        Returns:
            Plane: the copied plane
        """
        return Plane(normal=self._normal,
                     distance=self.distance)

    def __eq__(self, other: 'Plane') -> bool:
        """
        Equality of Plane.

        Args:
            other (Plane): other to compare

        Returns:
            bool: True if the differences of all components are within epsilon, Otherwise False.
        """
        return self._normal == other._normal and self._constant == other._constant

    def __ne__(self, other: 'Plane') -> bool:
        """
        Inequality of points is inequality of any coordinates

        Args:
            other (Plane): other to compare

        Returns:
            bool: False if the differences of all components are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __str__(self) -> str:
        """
        String representation of a plane

        Returns:
            str: String representation of a plane
        """
        return "(normal=%s, distance=%s)" % (repr(self._normal), repr(self.distance))

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Plane" + str(self)
