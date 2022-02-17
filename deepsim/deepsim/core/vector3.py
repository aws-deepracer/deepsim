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
"""A class for vector3."""
from typing import Union, Tuple, TypeVar, Optional, Iterable, Iterator
import math

import numpy as np
from shapely.geometry import Point as ShapelyPoint

from geometry_msgs.msg import Vector3 as ROSVector3

Point = TypeVar('Point')
Quaternion = TypeVar('Quaternion')


class Vector3:
    """
    Vector3 class
    """
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, *,
                 buffer: Optional[Iterable[float]] = None) -> None:
        """
        Initialize Vector3 class

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

    @staticmethod
    def one() -> 'Vector3':
        """
        Returns the one vector.

        Returns:
            Vector3: the one vector.
        """
        return Vector3(1.0, 1.0, 1.0)

    @staticmethod
    def zero() -> 'Vector3':
        """
        Returns the zero vector.

        Returns:
            Vector3: the zero vector.
        """
        return Vector3(0.0, 0.0, 0.0)

    @staticmethod
    def forward() -> 'Vector3':
        """
        Returns the forward vector.

        Returns:
            Vector3: the forward vector.
        """
        from deepsim.gazebo.constants import GazeboWorld
        return GazeboWorld.FORWARD.vector

    @staticmethod
    def back() -> 'Vector3':
        """
        Returns the backward vector.

        Returns:
            Vector3: the backward vector.
        """
        from deepsim.gazebo.constants import GazeboWorld
        return GazeboWorld.BACK.vector

    @staticmethod
    def left() -> 'Vector3':
        """
        Returns the left vector.

        Returns:
            Vector3: the left vector.
        """
        from deepsim.gazebo.constants import GazeboWorld
        return GazeboWorld.LEFT.vector

    @staticmethod
    def right() -> 'Vector3':
        """
        Returns the right vector.

        Returns:
            Vector3: the right vector.
        """
        from deepsim.gazebo.constants import GazeboWorld
        return GazeboWorld.RIGHT.vector

    @staticmethod
    def up() -> 'Vector3':
        """
        Returns the up vector.

        Returns:
            Vector3: the up vector.
        """
        from deepsim.gazebo.constants import GazeboWorld
        return GazeboWorld.UP.vector

    @staticmethod
    def down() -> 'Vector3':
        """
        Returns the down vector.

        Returns:
            Vector3: the down vector.
        """
        from deepsim.gazebo.constants import GazeboWorld
        return GazeboWorld.DOWN.vector

    @staticmethod
    def lerp(a: Union['Vector3', Point], b: Union['Vector3', Point], t: float) -> 'Vector3':
        """
        Linear interpolated between quaternions a and b.

        Args:
            a (Union['Vector3', Point]): start value
            b (Union['Vector3', Point]): end value
            t (float): fraction

        Returns:
            Vector3: interpolated vector3
        """
        from deepsim.core.point import Point
        a = a.to_vector() if isinstance(a, Point) else a
        b = b.to_vector() if isinstance(b, Point) else b
        t = np.clip(t, 0.0, 1.0).item()
        return Vector3(x=a.x + t * (b.x - a.x),
                       y=a.y + t * (b.y - a.y),
                       z=a.z + t * (b.z - a.z))

    @staticmethod
    def slerp(a: Union['Vector3', Point], b: Union['Vector3', Point], t: float) -> 'Vector3':
        """
        Spherically interpolated between vector a and b.

        Args:
            a (Union['Vector3', Point]): start value
            b (Union['Vector3', Point]): end value
            t (float): fraction

        Returns:
            Vector3: interpolated vector3
        """
        from deepsim.core.point import Point
        a = a.to_vector() if isinstance(a, Point) else a
        b = b.to_vector() if isinstance(b, Point) else b
        t = np.clip(t, 0.0, 1.0).item()

        dot = np.clip(a.dot(b), -1.0, 1.0).item()
        theta = math.acos(dot) * t
        relative_vec = (b - a * dot).norm()
        return (a * math.cos(theta)) + (relative_vec * math.sin(theta))

    @property
    def x(self) -> float:
        """
        Returns x value of the vector.

        Returns:
            float: x value of the vector.
        """
        return self._buffer[0]

    @x.setter
    def x(self, value: float) -> None:
        """
        Set new x value of the vector.

        Args:
            value (float): new x value.
        """
        self._buffer[0] = value

    @property
    def y(self) -> float:
        """
        Returns y value of the vector.

        Returns:
            float: y value of the vector.
        """
        return self._buffer[1]

    @y.setter
    def y(self, value: float) -> None:
        """
        Set new y value of the vector.

        Args:
            value (float): new y value.
        """
        self._buffer[1] = value

    @property
    def z(self) -> float:
        """
        Returns z value of the vector.

        Returns:
            float: z value of the vector.
        """
        return self._buffer[2]

    @z.setter
    def z(self, value: float) -> None:
        """
        Set new z value of the vector.

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

    @property
    def sqr_magnitude(self) -> float:
        """
        Returns the squared magnitude of the vector.

        Returns:
            float: the squared magnitude of the vector.
        """
        return self.dot(self)

    @property
    def magnitude(self) -> float:
        """
        Returns the magnitude of the vector.

        Returns:
            float: the magnitude of the vector.
        """
        return math.sqrt(self.sqr_magnitude)

    @staticmethod
    def _rotate_vector(v: 'Vector3', q: Quaternion) -> Tuple[float, float, float]:
        """
        Returns the rotated vector in the orientation of the given quaternion.

        This function assumes that v is a homogeneous quaternion. That is the real part is zero.
        The complete explanation can be found in the link
        https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
        https://en.wikipedia.org/wiki/Quaternion#Hamilton_product

        On an high level. We want the vector v in the direction of the quaternion q. We know that
        q * q_conj = 1

        p = q * v * q_conj, where p is pure quaternion, same length as v in the direction of q.

        The simplified formula in the executed code is derived from the below equations

        quaternion_mult(q,r)
            b1, c1, d1, a1 = q  # Here a1 and a2 are real numbers, b1, c1, d1 are imaginary i,j,k
            b2, c2, d2, a2 = r
            return [
                a1*b2 + b1*a2 + c1*d2 - d1*c2,
                a1*c2 - b1*d2 + c1*a2 + d1*b2,
                a1*d2 + b1*c2 - c1*b2 + d1*a2,
                a1*a2 - b1*b2 - c1*c2 - d1*d2
            ]

        rotate(v, q):
            r = np.insert(v, 3, 0)
            q_conj = [-1*q[0],-1*q[1],-1*q[2], q[3]]
            return quaternion_mult(quaternion_mult(q,r), q_conj)[:3]

        If the vector is not pure quaternion. Then in the below simplified solution the real value returned will be
        a2*( a1_sq + b1_sq + c1_sq + d1_sq)

        Args:
            v (Vector3): vector to apply the given quaternion.
            q (Quaternion): A quaternion

        Returns:
            Tuple[float, float, float]: final vector from v with q applied.
        """
        b1, c1, d1, a1 = q.x, q.y, q.z, q.w
        b2, c2, d2 = v.x, v.y, v.z

        a1_sq = a1 ** 2
        b1_sq = b1 ** 2
        c1_sq = c1 ** 2
        d1_sq = d1 ** 2

        x = b2 * (-c1_sq - d1_sq + b1_sq + a1_sq) + 2 * \
            (-(a1 * c2 * d1) + (b1 * c1 * c2) + (b1 * d1 * d2) + (a1 * c1 * d2))
        y = c2 * (c1_sq - d1_sq + a1_sq - b1_sq) + 2 * \
            ((a1 * b2 * d1) + (b1 * b2 * c1) + (c1 * d1 * d2) - (a1 * b1 * d2))
        z = d2 * (-c1_sq + d1_sq + a1_sq - b1_sq) + 2 * \
            ((a1 * b1 * c2) + (b1 * b2 * d1) - (a1 * b2 * c1) + (c1 * c2 * d1))
        return x, y, z

    @staticmethod
    def rotate_vector(v: 'Vector3', q: Quaternion) -> 'Vector3':
        """
        Rotate the given vector by given quaternion.

        Args:
            v (Vector3): vector to apply the given quaternion.
            q (Quaternion): A quaternion

        Returns:
            Vector3: rotated vector.
        """
        return Vector3.from_list(Vector3._rotate_vector(v, q))

    @staticmethod
    def project(v: 'Vector3', on_normal: 'Vector3') -> 'Vector3':
        """
        Project vector v onto on_normal:
        - The projection is just on_normal rescaled to that it reaches that point on the line v.
        Args:
            v (Vector3): vector to project to on_normal
            on_normal (Vector3): direction vector

        Returns:
            Vector3: projection in Vector3 format
        """
        sqr_mag = on_normal.sqr_magnitude
        if np.isclose(sqr_mag, 0.0):
            return Vector3(0.0, 0.0, 0.0)
        else:
            dot = v.dot(on_normal)
            return Vector3(on_normal.x * dot / sqr_mag,
                           on_normal.y * dot / sqr_mag,
                           on_normal.z * dot / sqr_mag)

    def rotate(self, q: Quaternion) -> 'Vector3':
        """
        Returns the rotated vector in the orientation of the given quaternion.

        Args:
            q (Quaternion): A quaternion

        Returns:
            Vector3: final vector from v with q applied.
        """
        return self.rotate_vector(v=self, q=q)

    def rotate_inplace(self, q: Quaternion) -> None:
        """
        Rotate the vector in the orientation of the given quaternion in place.

        Args:
            q (Quaternion): A quaternion
        """
        self._buffer = np.array(self._rotate_vector(v=self, q=q))

    def to_ros(self) -> ROSVector3:
        """
        Convert to ROS model

        Returns:
            geometry_msgs.msg.Vector3: ROS Vector3
        """
        ros_vector3 = ROSVector3()
        ros_vector3.x = self.x
        ros_vector3.y = self.y
        ros_vector3.z = self.z
        return ros_vector3

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

    def to_point(self) -> Point:
        """
        Convert to Point

        Returns:
            Point: Point containing x, y, z
        """
        from deepsim.core.point import Point
        return Point(buffer=self._buffer)

    @staticmethod
    def from_ros(value: ROSVector3) -> 'Vector3':
        """
        Returns new Vector3 object created from ROS Vector3

        Args:
            value (geometry_msgs.msg.Vector3): ROS Vector3

        Returns:
            Vector3: new Vector3 object created from ROS Vector3
        """
        return Vector3(x=value.x,
                       y=value.y,
                       z=value.z)

    @staticmethod
    def from_list(value: Union[list, tuple]) -> 'Vector3':
        """
        Return new Vector3 object from list

        Args:
            value (Union[list, tuple]): Vector3 in list or tuple type

        Returns:
            Vector3: new Vector3 object created from list
        """
        return Vector3(buffer=value)

    @staticmethod
    def from_numpy(value: np.ndarray) -> 'Vector3':
        """
        Return new Vector3 object from numpy.array

        Args:
            value (np.ndarray): Vector3 in numpy.array type

        Returns:
            Vector3: new Vector3 object created from numpy.array
        """
        return Vector3(buffer=value)

    @staticmethod
    def from_point(value: Point) -> 'Vector3':
        """
        Return new Vector3 object from numpy.array

        Args:
            value (Point): Point

        Returns:
            Vector3: new Vector3 object created from numpy.array
        """
        return Vector3(buffer=value.buffer)

    @staticmethod
    def from_shapely(value: ShapelyPoint) -> 'Vector3':
        """
        Return new Vector3 object from ShapelyPoint

        Args:
            value (ShapelyPoint): shapely point.

        Returns:
            Vector3: new Vector3 object created from ShapelyPoint
        """
        return Vector3(x=value.x,
                       y=value.y,
                       z=value.z if value.has_z else 0.0)

    def copy(self) -> 'Vector3':
        """
        Returns a copy.

        Returns:
            Vector3: the copied vector
        """
        return Vector3(buffer=self.buffer)

    def __add__(self, other: Union[Point, 'Vector3']) -> 'Vector3':
        """
        Returns a vector sum.

        Args:
            other (Union[Point, 'Vector3']): other to add.

        Returns:
            Vector3: a vector sum.
        """
        from deepsim.core.point import Point
        if isinstance(other, Point) or isinstance(other, Vector3):
            return Vector3(buffer=self.buffer + other.buffer)
        return NotImplemented

    def __sub__(self, other: Union[Point, 'Vector3']) -> 'Vector3':
        """
        Returns a vector difference.

        Args:
            other (Union[Point, 'Vector3']): other to subtract.

        Returns:
            Vector3: a vector difference.
        """
        from deepsim.core.point import Point
        if isinstance(other, Point) or isinstance(other, Vector3):
            return Vector3(buffer=self.buffer - other.buffer)
        return NotImplemented

    def __mul__(self, other: Union[float, int, 'Vector3', Point]) -> 'Vector3':
        """
        Returns scale or element multiplication, v * scale or v * v or v * p.

        Args:
            other (Union[float, int, 'Vector3', Point]): scale

        Returns:
            Vector3: v * scale
        """
        from deepsim.core.point import Point
        if isinstance(other, float) or isinstance(other, int):
            return Vector3(buffer=self.buffer * other)
        if isinstance(other, Vector3) or isinstance(other, Point):
            return Vector3(buffer=self.buffer * other.buffer)
        return NotImplemented

    def __rmul__(self, other: Union[float, int, Quaternion]) -> 'Vector3':
        """
        Returns scale multiplication, scale * v or Q * v.

        Args:
            other (Union[float, int, Quaternion]): scale or quaternion.

        Returns:
            Vector3: scale * v or Q * v
        """
        from deepsim.core.quaternion import Quaternion
        if isinstance(other, float) or isinstance(other, int):
            return self.__mul__(other)
        if isinstance(other, Quaternion):
            return self.rotate(other)
        return NotImplemented

    def __truediv__(self, scale: Union[float, int]) -> 'Vector3':
        """
        Returns a vector resulted by v / scale
        - division of a vector by a float r is scaling by (1/r)

        Args:
            scale (Union[float, int]): scale

        Returns:
            Vector3: v / scale
        """
        if isinstance(scale, float) or isinstance(scale, int):
            return self.__mul__(1.0 / scale)
        return NotImplemented

    def __iadd__(self, other: Union[Point, 'Vector3']) -> 'Vector3':
        """
        Assign the vector sum.

        Args:
            other (Union[Point, 'Vector3']): other to add.

        Returns:
            Vector3: self += other
        """
        from deepsim.core.point import Point
        if isinstance(other, Vector3) or isinstance(other, Point):
            self._buffer += other.buffer
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __isub__(self, other: Union[Point, 'Vector3']) -> 'Vector3':
        """
        Assign the vector difference.

        Args:
            other (Union[Point, 'Vector3']): other to subtract.

        Returns:
            Vector3: self -= other
        """
        from deepsim.core.point import Point
        if isinstance(other, Vector3) or isinstance(other, Point):
            self._buffer -= other.buffer
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __imul__(self, other: Union[float, int, 'Vector3', Point]) -> 'Vector3':
        """
        Assign other multiplication, v * scale, v * v, v * p.

        Args:
            other (Union[float, int, 'Vector3', Point]): scale, vector, or point

        Returns:
            Vector3: self *= other
        """
        from deepsim.core.point import Point
        if isinstance(other, Vector3) or isinstance(other, Point):
            self._buffer *= other.buffer
        elif isinstance(other, float) or isinstance(other, int):
            self._buffer *= other
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __idiv__(self, scale: Union[float, int]) -> 'Vector3':
        """
        Assign a vector resulted by v / scale
        - division of a vector by a float r is scaling by (1/r)

        Args:
            scale (Union[float, int]): scale

        Returns:
            Vector3: self /= other
        """
        if isinstance(scale, float) or isinstance(scale, int):
            return self.__imul__(1.0 / scale)
        else:
            raise ValueError("Not supported type {}.".format(type(scale)))

    def __neg__(self) -> 'Vector3':
        """
        Returns negated vector.
         - The negation of a vector is negation of all its coordinates

        Returns:
            Vector3: the negated vector
        """
        return Vector3(buffer=-self.buffer)

    def __iter__(self) -> Iterator[float]:
        """
        Iterator over the coordinates

        Returns:
            Iterator[float]: iterator
        """
        return self.buffer.__iter__()

    def __eq__(self, other: 'Vector3') -> bool:
        """
        Equality of vector is equality of all coordinates to within epsilon.

        Args:
            other (Vector3): other to compare

        Returns:
            bool: True if the differences of all coordinates are within epsilon, Otherwise False.
        """
        return np.all(np.isclose(self.buffer, other.buffer))

    def __ne__(self, other: 'Vector3') -> bool:
        """
        Inequality of vector is inequality of any coordinates

        Args:
            other (Vector3): other to compare

        Returns:
            bool: False if the differences of all coordinates are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __getitem__(self, i: int) -> float:
        """
        Return V[i] where V[i] is x, y, z for i in 0, 1, 2 respectively.

        Args:
            i (int): index

        Returns:
            float: value at the given index.
        """
        return self.buffer[i]

    def __setitem__(self, i: int, value: float) -> None:
        """
        Set V[i] where V[i] is x, y, z for i in 0, 1, 2 respectively.
        Args:
            i (int): index
            value (float): new value
        """
        self.buffer[i] = value

    def distance(self, other: Union[Point, 'Vector3']) -> float:
        """
        Returns the distance between self and other.
        - (other - self).magnitude

        Args:
            other (Vector3): other vector

        Returns:
            float: (other - self).magnitude
        """
        return (other - self).magnitude

    def cross(self, other: Union[Point, 'Vector3']) -> 'Vector3':
        """
        Returns the cross product.

        Args:
            other (Vector3): other to dot product.

        Returns:
            Vector3: the result of cross product
        """
        return Vector3.from_numpy(np.cross(self.buffer, other.buffer))

    def dot(self, other: Union[Point, 'Vector3']) -> float:
        """
        Returns the dot product.

        Args:
            other (Vector3): other to dot product.

        Returns:
            float: the result of dot product
        """
        return float(np.dot(self.buffer, other.buffer))

    def _norm(self) -> np.ndarray:
        """
        Normalize this vector

        Returns:
            np.ndarray: unit vector
        """
        norm = np.linalg.norm(self._buffer)
        if norm == 0:
            return self._buffer
        unit_vector = self._buffer / norm
        return unit_vector

    def norm(self) -> 'Vector3':
        """
        Returns the normalized vector

        Returns:
            Vector3: unit vector
        """
        return Vector3.from_numpy(self._norm())

    def normalize(self) -> 'Vector3':
        """
        Normalize the vector in place

        Returns:
            Vector3: normalized self (Vector3) object
        """
        self._buffer = self._norm()
        return self

    def __str__(self) -> str:
        """
        String representation of a vector3

        Returns:
            str: String representation of a vector3
        """
        return "(%f,%f,%f)" % (self.x, self.y, self.z)

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Vector3" + str(self)
