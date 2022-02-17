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
"""A class for ray."""
from typing import Optional, Union
from deepsim.core.vector3 import Vector3
from deepsim.core.point import Point


class Ray:
    """
    Ray class
    """
    def __init__(self,
                 origin: Optional[Union[Vector3, Point]] = None,
                 direction: Optional[Vector3] = None) -> None:
        """
        Initialize Ray class

        Args:
            origin (Optional[Union[Vector3, Point]]): origin of ray.
            direction (Optional[Vector3]): ray direction.
        """
        if origin and isinstance(origin, Point):
            origin = origin.to_vector()
        self._origin = origin.copy() if origin else Vector3()
        self._direction = direction.norm() if direction else Vector3()

    @property
    def origin(self) -> Vector3:
        """
        Returns the copy of origin vector.

        Returns:
            Vector3: the copy of origin vector.
        """
        return self._origin.copy()

    @origin.setter
    def origin(self, value: Union[Vector3, Point]) -> None:
        """
        Set the new origin vector.

        Args:
            value (Union[Vector3, Point]): the new origin.
        """
        if isinstance(value, Point):
            value = value.to_vector()
        self._origin = value.copy()

    @property
    def direction(self) -> Vector3:
        """
        Returns the copy of ray direction.

        Returns:
            Vector3: the copy of ray direction.
        """
        return self._direction.copy()

    @direction.setter
    def direction(self, value: Vector3) -> None:
        """
        Set the new ray direction.

        Args:
            value (Vector3): the new ray direction.
        """
        self._direction = value.norm()

    def get_point(self, distance: float) -> Vector3:
        """
        Returns a point at distance units along the ray.

        Args:
            distance (float): the distance

        Returns:
            Vector3: a point at distance units along the ray.
        """
        return Vector3.from_numpy(self._direction.buffer * distance + self._origin.buffer)

    def __eq__(self, other: 'Ray') -> bool:
        """
        Equality of Ray.

        Args:
            other (Ray): other to compare

        Returns:
            bool: True if the differences of all components are within epsilon, Otherwise False.
        """
        return self._direction == other._direction and self._origin == other._origin

    def __ne__(self, other: 'Ray') -> bool:
        """
        Inequality of points is inequality of any coordinates

        Args:
            other (Ray): other to compare

        Returns:
            bool: False if the differences of all components are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __str__(self) -> str:
        """
        String representation of a ray

        Returns:
            str: String representation of a ray
        """
        return "(origin=%s, direction=%s)" % (repr(self._origin), repr(self._direction))

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Ray" + str(self)
