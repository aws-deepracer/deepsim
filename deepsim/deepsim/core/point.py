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
"""A class for point."""
from typing import Union, TypeVar, Iterable, Optional, Iterator
import math

import numpy as np
from shapely.geometry import Point as ShapelyPoint

from geometry_msgs.msg import Point as ROSPoint


Vector3 = TypeVar('Vector3')
Quaternion = TypeVar('Quaternion')


class Point:
    """
    Point class
    """
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, *,
                 buffer: Optional[Iterable[float]] = None):
        """
        Constructor

        Args:
            x (float): x value
            y (float): y value
            z (float): z value
            buffer (Optional[Iterable[float]]): buffer to copy directly to internal representation.
        """
        buffer = buffer if buffer is not None else [x, y, z]
        if len(buffer) < 3:
            raise ValueError("buffer must contain at least 3 elements")
        self._buffer = np.array(buffer[:3], dtype=float)

    @property
    def x(self) -> float:
        """
        Returns x value of the point.

        Returns:
            float: x value of the point.
        """
        return self._buffer[0]

    @x.setter
    def x(self, value: float) -> None:
        """
        Set new x value of the point.

        Args:
            value (float): new x value.
        """
        self._buffer[0] = value

    @property
    def y(self) -> float:
        """
        Returns y value of the point.

        Returns:
            float: y value of the point.
        """
        return self._buffer[1]

    @y.setter
    def y(self, value: float) -> None:
        """
        Set new y value of the point.

        Args:
            value (float): new y value.
        """
        self._buffer[1] = value

    @property
    def z(self) -> float:
        """
        Returns z value of the point.

        Returns:
            float: z value of the point.
        """
        return self._buffer[2]

    @z.setter
    def z(self, value: float) -> None:
        """
        Set new z value of the point.

        Args:
            value (float): new z value.
        """
        self._buffer[2] = value

    @property
    def buffer(self) -> np.ndarray:
        """
        Returns internal buffer.
        - Use with caution as changing the returned buffer will result changing the object.

        Returns:
            np.ndarray: internal buffer.
        """
        return self._buffer

    @staticmethod
    def rotate_point(p: 'Point', q: Quaternion) -> 'Point':
        """
        Rotate the given point by given quaternion.

        Args:
            p (Point): point to apply the given quaternion.
            q (Quaternion): A quaternion

        Returns:
            Point: rotated point.
        """
        return p.to_vector().rotate(q).to_point()

    @staticmethod
    def project(p: 'Point', on_normal: Vector3) -> 'Point':
        """
        Project point p onto on_normal:
        - The projection is just on_normal rescaled to that it reaches that point on the line v.
        Args:
            p (Point): point to project to on_normal
            on_normal (Vector3): direction vector

        Returns:
            Point: projection in Point format
        """
        from deepsim.core.vector3 import Vector3
        return Vector3.project(p.to_vector(), on_normal).to_point()

    def rotate(self, q: Quaternion) -> 'Point':
        """
        Returns the rotated point in the orientation of the given quaternion.

        Args:
            q (Quaternion): A quaternion

        Returns:
            Point: final point from p with q applied.
        """
        return self.to_vector().rotate(q).to_point()

    def rotate_inplace(self, q: Quaternion) -> None:
        """
        Rotate the point in the orientation of the given quaternion in place.

        Args:
            q (Quaternion): A quaternion
        """
        self._buffer = self.to_vector().rotate(q).buffer

    def to_ros(self) -> ROSPoint:
        """
        Convert to ROS model

        Returns:
            geometry_msgs.msg.Vector3: ROS Point
        """
        ros_point = ROSPoint()
        ros_point.x = self.x
        ros_point.y = self.y
        ros_point.z = self.z
        return ros_point

    def to_list(self) -> list:
        """
        Convert to Python list

        Returns:
            list: list containing x, y, z in order.
        """
        return list(self._buffer)

    def to_numpy(self) -> np.ndarray:
        """
        Convert to Numpy array

        Returns:
            numpy.array: numpy array containing x, y, z in order.
        """
        return self._buffer.copy()

    def to_vector(self) -> Vector3:
        """
        Convert to Vector3

        Returns:
            Vector3: Vector3 containing x, y, z
        """
        from deepsim.core.vector3 import Vector3
        return Vector3(buffer=self._buffer)

    def to_shapely(self) -> ShapelyPoint:
        """
        Convert to Shapely

        Returns:
            ShapelyPoint: shapely Point containing x, y, z
        """
        return ShapelyPoint(self.buffer)

    def to_shapely_2d(self) -> ShapelyPoint:
        """
        Convert to Shapely

        Returns:
            ShapelyPoint: shapely Point containing x, y
        """
        return ShapelyPoint(self.buffer[0:2])

    @staticmethod
    def from_ros(value: ROSPoint) -> 'Point':
        """
        Returns new Point object created from ROS Point

        Args:
            value (geometry_msgs.msg.Point): ROS Point

        Returns:
            ROSPoint: new ROSPoint object created from ROS Point
        """
        return Point(x=value.x,
                     y=value.y,
                     z=value.z)

    @staticmethod
    def from_list(value: Union[list, tuple]) -> 'Point':
        """
        Return new Point object from list

        Args:
            value (Union[list, tuple]): Point in list or tuple type

        Returns:
            Point: new Point object created from list
        """
        return Point(buffer=value)

    @staticmethod
    def from_numpy(value: np.ndarray) -> 'Point':
        """
        Return new Point object from numpy.array

        Args:
            value (np.ndarray): Point in numpy.array type

        Returns:
            Point: new Point object created from numpy.array
        """
        return Point(buffer=value)

    @staticmethod
    def from_vector(value: Vector3) -> 'Point':
        """
        Return new Point object from Vector3

        Args:
            value (Vector3): Vector3 object

        Returns:
            Point: new Point object created from Vector3
        """
        return Point(buffer=value.buffer)

    @staticmethod
    def from_shapely(value: ShapelyPoint) -> 'Point':
        """
        Return new Point object from ShapelyPoint

        Args:
            value (ShapelyPoint): shapely point.

        Returns:
            Point: new Point object created from ShapelyPoint
        """
        return Point(x=value.x,
                     y=value.y,
                     z=value.z if value.has_z else 0.0)

    @staticmethod
    def get_angle_in_2d_rad(pt1: 'Point', pt2: 'Point') -> float:
        """
            Return angle between two points in radian measured counter-clockwise from the +x axis.

            Args:
                pt1 (Point): point 1
                pt2 (Point): point 2

            Returns:
                float: the angle between two points in radian measured counter-clockwise from the +x axis.
            """
        return math.atan2(pt2.y - pt1.y, pt2.x - pt1.x)

    def copy(self) -> 'Point':
        """
        Returns a copy.

        Returns:
            Point: the copied point
        """
        return Point(buffer=self.buffer)

    def __add__(self, other: Union['Point', Vector3]) -> Union['Point', Vector3]:
        """
        Returns a Point from the addition of self and other.
        - P + v is P translated by v

        Args:
            other (Union[Point, Vector3]): other to subtract

        Returns:
            Union['Point', Vector3]: a Point from the addition of self and other
        """
        from deepsim.core.vector3 import Vector3
        if isinstance(other, Vector3):
            return Point(buffer=self.buffer + other.buffer)
        elif isinstance(other, Point):
            return Vector3(buffer=self.buffer + other.buffer)
        else:
            return NotImplemented

    def __sub__(self, other: Union['Point', Vector3]) -> Union['Point', Vector3]:
        """
        Returns a Point from the subtraction of other from self.

        Args:
            other (Point): other to subtract

        Returns:
            Point: a Point from the subtraction of other from self.
        """
        from deepsim.core.vector3 import Vector3
        if isinstance(other, Vector3):
            return Point(buffer=self.buffer - other.buffer)
        elif isinstance(other, Point):
            return Vector3(buffer=self.buffer - other.buffer)
        else:
            return NotImplemented

    def __mul__(self, other: Union[float, int, 'Point', Vector3]) -> 'Point':
        """
        Returns scale or element multiplication, p * scale or p * v or p * p.

        Args:
            other (Union[float, int, Vector3, Point]): scale

        Returns:
            Vector3: v * scale
        """
        from deepsim.core.vector3 import Vector3
        if isinstance(other, float) or isinstance(other, int):
            return Point(buffer=self.buffer * other)
        if isinstance(other, Vector3) or isinstance(other, Point):
            return Point(buffer=self.buffer * other.buffer)
        return NotImplemented

    def __rmul__(self, other: Union[float, int, Quaternion]) -> 'Point':
        """
        Returns scale multiplication, scale * p or Q * p.

        Args:
            other (Union[float, int, Quaternion]): scale or quaternion.

        Returns:
            Vector3: scale * v or Q * v
        """
        from deepsim.core.quaternion import Quaternion
        if isinstance(other, float) or isinstance(other, int):
            return self.__mul__(other)
        if isinstance(other, Quaternion):
            return self.to_vector().rotate(other).to_point()
        return NotImplemented

    def __truediv__(self, scale: Union[float, int]) -> 'Point':
        """
        Returns a point resulted by p / scale
        - division of a point by a float r is scaling by (1/r)

        Args:
            scale (Union[float, int]): scale

        Returns:
            Point: p / scale
        """
        if isinstance(scale, float) or isinstance(scale, int):
            return self.__mul__(1.0 / scale)
        return NotImplemented

    def __iadd__(self, other: Union['Point', Vector3]) -> 'Point':
        """
        Assign the point sum.

        Args:
            other (Vector3): other to add.

        Returns:
            Point: self += other
        """
        from deepsim.core.vector3 import Vector3
        if isinstance(other, Vector3) or isinstance(other, Point):
            self._buffer += other.buffer
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __isub__(self, other: Union['Point', Vector3]) -> 'Point':
        """
        Assign the point difference.

        Args:
            other (Vector3): other to subtract.

        Returns:
            Point: p -= v
        """
        from deepsim.core.vector3 import Vector3
        if isinstance(other, Vector3) or isinstance(other, Point):
            self._buffer -= other.buffer
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __imul__(self, other: Union[float, int, 'Point', Vector3]) -> 'Point':
        """
        Assign other multiplication, p *= scale, p *= p, p *= v.

        Args:
            other (Union[float, int, 'Point', Vector3]): scale, vector, or point

        Returns:
            Point: p *= scale, p *= p, p *= v
        """
        from deepsim.core.vector3 import Vector3
        if isinstance(other, Vector3) or isinstance(other, Point):
            self._buffer *= other.buffer
        elif isinstance(other, float) or isinstance(other, int):
            self._buffer *= other
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __idiv__(self, scale: Union[float, int]) -> 'Point':
        """
        Assign a point resulted by p / scale
        - division of a point by a float r is scaling by (1/r)

        Args:
            scale (Union[float, int]): scale

        Returns:
            Point: p /= scale
        """
        if isinstance(scale, float) or isinstance(scale, int):
            return self.__imul__(1.0 / scale)
        else:
            raise ValueError("Not supported type {}.".format(type(scale)))

    def __neg__(self) -> 'Point':
        """
        Returns negated point.
         - The negation of a point is negation of all its coordinates

        Returns:
            Point: the negated vector
        """
        return Point(buffer=-self.buffer)

    def __iter__(self) -> Iterator[float]:
        """
        Iterator over the coordinates

        Returns:
            Iterator[float]: iterator
        """
        return self.buffer.__iter__()

    def __eq__(self, other: 'Point') -> bool:
        """
        Equality of point is equality of all coordinates to within epsilon.

        Args:
            other (Point): other to compare

        Returns:
            bool: True if the differences of all coordinates are within epsilon, Otherwise False.
        """
        return np.all(np.isclose(self.buffer, other.buffer))

    def __ne__(self, other: 'Point') -> bool:
        """
        Inequality of point is inequality of any coordinates

        Args:
            other (Point): other to compare

        Returns:
            bool: False if the differences of all coordinates are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __getitem__(self, i: int) -> float:
        """
        Return P[i] where P[i] is x, y, z for i in 0, 1, 2 respectively.

        Args:
            i (int): index

        Returns:
            float: value at the given index.
        """
        return self.buffer[i]

    def __setitem__(self, i: int, value: float) -> None:
        """
        Set P[i] where P[i] is x, y, z for i in 0, 1, 2 respectively.
        Args:
            i (int): index
            value (float): new value
        """
        self.buffer[i] = value

    def __str__(self) -> str:
        """
        String representation of a point

        Returns:
            str: String representation of a point
        """
        return "(%f,%f,%f)" % (self.x, self.y, self.z)

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Point" + str(self)
