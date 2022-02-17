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
"""A class for quaternion."""
import math
from typing import TypeVar, Union, Optional, Iterable, Iterator

from deepsim.core.math import euler_to_quaternion, quaternion_to_euler

import numpy as np

from geometry_msgs.msg import Quaternion as ROSQuaternion

Euler = TypeVar('Euler')
Vector3 = TypeVar('Vector3')
Point = TypeVar('Point')


class Quaternion:
    """
    Quaternion class
    """
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0, *,
                 buffer: Optional[Iterable[float]] = None) -> None:
        """
        Initialize Quaternion class

        Args:
            x (float): x value
            y (float): y value
            z (float): z value
            w (float): w value
            buffer (Optional[Iterable[float]]): buffer to copy directly to internal representation.
        """
        buffer = buffer if buffer is not None else [x, y, z, w]
        if len(buffer) < 4:
            raise ValueError("buffer must contain at least 4 elements")
        self._buffer = np.array(buffer[:4], dtype=float)

    @property
    def x(self) -> float:
        """
        Returns x value of the quaternion.

        Returns:
            float: x value of the quaternion.
        """
        return self._buffer[0]

    @x.setter
    def x(self, value: float) -> None:
        """
        Set new x value of the quaternion.

        Args:
            value (float): new x value.
        """
        self._buffer[0] = value

    @property
    def y(self) -> float:
        """
        Returns y value of the quaternion.

        Returns:
            float: y value of the quaternion.
        """
        return self._buffer[1]

    @y.setter
    def y(self, value: float) -> None:
        """
        Set new y value of the quaternion.

        Args:
            value (float): new y value.
        """
        self._buffer[1] = value

    @property
    def z(self) -> float:
        """
        Returns z value of the quaternion.

        Returns:
            float: z value of the quaternion.
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
    def w(self) -> float:
        """
        Returns w value of the quaternion.

        Returns:
            float: w value of the quaternion.
        """
        return self._buffer[3]

    @w.setter
    def w(self, value: float) -> None:
        """
        Set new w value of the point.

        Args:
            value (float): new w value.
        """
        self._buffer[3] = value

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
        Returns the squared magnitude of the quaternion.

        Returns:
            float: the squared magnitude of the quaternion.
        """
        return self.dot(self)

    @property
    def magnitude(self) -> float:
        """
        Returns the magnitude of the quaternion.

        Returns:
            float: the magnitude of the quaternion.
        """
        return math.sqrt(self.sqr_magnitude)

    @staticmethod
    def identity() -> 'Quaternion':
        """
        Returns identity in Quaternion.

        Returns:
            Quaternion: identity.
        """
        return Quaternion(0.0, 0.0, 0.0, 1.0)

    def to_ros(self) -> ROSQuaternion:
        """
        Convert to ROS model

        Returns:
            geometry_msgs.msg.Quaternion: ROS Quaternion
        """
        ros_quaternion = ROSQuaternion()
        ros_quaternion.x = self.x
        ros_quaternion.y = self.y
        ros_quaternion.z = self.z
        ros_quaternion.w = self.w
        return ros_quaternion

    def to_list(self) -> list:
        """
        Convert to Python list

        Returns:
            list: list containing x, y, z, w in order.
        """
        return list(self._buffer)

    def to_numpy(self) -> np.ndarray:
        """
        Convert to Numpy array

        Returns:
            numpy.array: numpy array containing x, y, z, w in order.
        """
        return self._buffer.copy()

    def to_euler(self) -> Euler:
        """
        Returns new Euler object converted from this Quaternion.

        Returns:
            Euler: new Euler object converted from Quaternion.
        """
        from deepsim.core.euler import Euler
        euler = quaternion_to_euler(x=self.x, y=self.y, z=self.z, w=self.w)
        return Euler(buffer=euler)

    def inverse_inplace(self) -> None:
        """
        Inverse the quaternion inplace.
        """
        self._buffer = self._inverse()

    def inverse(self) -> 'Quaternion':
        """
        Return inverse of this quaternion.

        Returns:
            Quaternion: inverted quaternion.
        """
        return Quaternion.from_numpy(self._inverse())

    def _inverse(self) -> np.ndarray:
        """
        Returns the inverse of this quaternion.

        Returns:
            np.ndarray: inverted quaternion

        """
        q = self.to_numpy()
        n = np.dot(q, q)
        if np.isclose(n, 0.0):
            raise ValueError("Cannot calculate inverse with quaternion magnitude is 0")
        return np.array([-q[0] / n, -q[1] / n, -q[2] / n, q[3] / n], dtype=float)

    @staticmethod
    def lerp(a: 'Quaternion', b: 'Quaternion', t: float,
             use_shortest_path: bool = True) -> 'Quaternion':
        """
        Linear Interpolation

        Args:
            a (Quaternion): normalized quaternion start value.
            b (Quaternion): normalized quaternion end value.
            t (float): fraction.
            use_shortest_path (bool): flag whether to use shortest path or not.

        Returns:
            Quaternion: interpolated quaternion
        """
        t = np.clip(t, 0.0, 1.0).item()
        if use_shortest_path:
            dot = a.dot(b)
            if dot < 0.0:
                b = -b
        return (a * (1 - t) + b * t).normalize()

    @staticmethod
    def slerp(a: 'Quaternion', b: 'Quaternion', t: float,
              use_shortest_path: bool = True) -> 'Quaternion':
        """
        Spherically interpolated between quaternions a and b.

        Args:
            a (Quaternion): start value
            b (Quaternion): end value
            t (float): fraction
            use_shortest_path (bool): flag whether to use shortest path or not.

        Returns:
            Quaternion: interpolated quaternion
        """
        t = np.clip(t, 0.0, 1.0).item()
        dot = a.dot(b)

        if dot < 0.0:
            b = -b

        if abs(dot) < 0.95:
            if use_shortest_path:
                angle = math.acos(abs(dot))
            else:
                angle = math.acos(dot)
            return (a * math.sin(angle * (1.0 - t)) + b * math.sin(angle * t)) / math.sin(angle)
        # Fall back to lerp if the angle is small
        return Quaternion.lerp(a, b, t,
                               use_shortest_path=False)

    @staticmethod
    def look_rotation(forward: Union[Vector3, Point], upwards: Optional[Union[Vector3, Point]] = None) -> 'Quaternion':
        """
        Return a rotation with given forward and upwards directions.

        Args:
            forward (Union[Vector3, Point]): direction to look in
            upwards (Optional[Union[Vector3, Point]]): the vector that defines in which direction up is.

        Returns:
            Quaternion: a rotation with the specified forward and upwards directions.
                        - identity if forward or upwards magnitude is zero.
                        - identity if forward and upwards are colinear.
        """
        from deepsim.core.vector3 import Vector3
        from deepsim.core.point import Point

        forward = forward.to_vector() if isinstance(forward, Point) else forward
        upwards = upwards.to_vector() if isinstance(upwards, Point) else upwards

        forward = forward.norm()
        up = upwards.norm() if upwards else Vector3.up()

        if forward == Vector3.zero() or up == Vector3.zero() or forward == up:
            return Quaternion.identity()

        left = up.cross(forward).norm()
        up = forward.cross(left)

        m00 = forward.x
        m01 = forward.y
        m02 = forward.z
        m10 = left.x
        m11 = left.y
        m12 = left.z
        m20 = up.x
        m21 = up.y
        m22 = up.z

        trace = m00 + m11 + m22

        quaternion = Quaternion()
        if trace > 0:
            num = math.sqrt(trace + 1.0)
            quaternion.w = 0.5 * num
            num = 0.5 / num
            quaternion.x = (m12 - m21) * num
            quaternion.y = (m20 - m02) * num
            quaternion.z = (m01 - m10) * num
            return quaternion
        if m00 >= m11 and m00 >= m22:
            num = math.sqrt(1.0 + m00 - m11 - m22)
            quaternion.x = 0.5 * num
            num = 0.5 / num
            quaternion.w = (m12 - m21) * num
            quaternion.y = (m10 + m01) * num
            quaternion.z = (m20 + m02) * num
            return quaternion
        if m11 > m22:
            num = math.sqrt(1.0 + m11 - m00 - m22)
            quaternion.y = 0.5 * num
            num = 0.5 / num
            quaternion.w = (m20 - m02) * num
            quaternion.x = (m10 + m01) * num
            quaternion.z = (m21 + m12) * num
            return quaternion
        num = math.sqrt(1.0 + m22 - m00 - m11)
        quaternion.z = 0.5 * num
        num = 0.5 / num
        quaternion.w = (m01 - m10) * num
        quaternion.x = (m20 + m02) * num
        quaternion.y = (m21 + m12) * num

        return quaternion

    @staticmethod
    def from_ros(value: ROSQuaternion) -> 'Quaternion':
        """
        Returns new Quaternion object created from ROS Quaternion

        Args:
            value (ROSQuaternion): ROS Quaternion

        Returns:
            Quaternion: new Quaternion object created from ROS Quaternion
        """
        return Quaternion(x=value.x,
                          y=value.y,
                          z=value.z,
                          w=value.w)

    @staticmethod
    def from_list(value: Union[list, tuple]) -> 'Quaternion':
        """
        Returns new Quaternion object created from list

        Args:
            value (Union[list, tuple]): quaternion in list or tuple type

        Returns:
            Quaternion: new Quaternion object created from list
        """
        return Quaternion(buffer=value)

    @staticmethod
    def from_numpy(value: np.ndarray) -> 'Quaternion':
        """
        Returns new Quaternion object created from numpy.array

        Args:
            value (np.ndarray): quaternion in numpy.array

        Returns:
            Quaternion: new Quaternion object created from numpy.array
        """
        return Quaternion(buffer=value)

    @staticmethod
    def from_euler(value: Union[Euler, list, tuple, np.ndarray]) -> 'Quaternion':
        """
        Return new Quaternion object from euler.

        Args:
            value (Union[Euler, list, tuple, np.ndarray]): euler

        Returns:
            Quaternion: new Quaternion object created from euler
        """
        from deepsim.core.euler import Euler
        if isinstance(value, list) or isinstance(value, tuple):
            value = Euler.from_list(value=value)
        elif type(value).__module__ == np.__name__:
            value = Euler.from_numpy(value=value)
        q = euler_to_quaternion(roll=value.roll, pitch=value.pitch, yaw=value.yaw)
        return Quaternion(buffer=q)

    def copy(self) -> 'Quaternion':
        """
        Returns a copy.

        Returns:
            Quaternion: the copied quaternion
        """
        return Quaternion(buffer=self.buffer)

    def __add__(self, other: 'Quaternion') -> 'Quaternion':
        """
        Returns a quaternion sum.

        Args:
            other (Quaternion): other to add.

        Returns:
            Quaternion: a quaternion sum.
        """
        if isinstance(other, Quaternion):
            return Quaternion(buffer=self.buffer + other.buffer)
        return NotImplemented

    def __sub__(self, other: 'Quaternion') -> 'Quaternion':
        """
        Returns a quaternion difference.

        Args:
            other (Quaternion): other to subtract.

        Returns:
            Quaternion: a quaternion difference.
        """
        if isinstance(other, Quaternion):
            return Quaternion(buffer=self.buffer - other.buffer)
        return NotImplemented

    def __mul__(self, other: Union[float, int, 'Quaternion']) -> 'Quaternion':
        """
        Returns scale or Quaternion multiplication, q * scale or q * q'.

        Args:
            other (Union[float, int, 'Quaternion']): scale or quaternion

        Returns:
            Quaternion: q * scale or q * q'
        """
        if isinstance(other, Quaternion):
            q = Quaternion()

            q.x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
            q.y = self.w * other.y + self.y * other.w + self.z * other.x - self.x * other.z
            q.z = self.w * other.z + self.z * other.w + self.x * other.y - self.y * other.x
            q.w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
            return q
        elif isinstance(other, float) or isinstance(other, int):
            return Quaternion(buffer=self.buffer * other)
        return NotImplemented

    def __rmul__(self, scale: Union[float, int]) -> 'Quaternion':
        """
        Returns scale multiplication, scale * q.

        Args:
            scale (Union[float, int]): scale

        Returns:
            Quaternion: scale * q
        """
        if isinstance(scale, float) or isinstance(scale, int):
            return self.__mul__(scale)
        return NotImplemented

    def __truediv__(self, scale: Union[float, int]) -> 'Quaternion':
        """
        Returns a quaternion resulted by q / scale
        - division of a quaternion by a float r is scaling by (1/r)

        Args:
            scale (Union[float, int]): scale

        Returns:
            Quaternion: q / scale
        """
        if isinstance(scale, float) or isinstance(scale, int):
            return self.__mul__(1.0 / scale)
        return NotImplemented

    def __iadd__(self, other: 'Quaternion') -> 'Quaternion':
        """
        Assign the quaternion sum.

        Args:
            other (Quaternion): other to add.

        Returns:
            Quaternion: self += other_quaternion
        """
        if isinstance(other, Quaternion):
            self._buffer += other.buffer
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __isub__(self, other: 'Quaternion') -> 'Quaternion':
        """
        Assign the quaternion difference.

        Args:
            other (Quaternion): other to subtract.

        Returns:
            Quaternion: self -= other_quaternion
        """
        if isinstance(other, Quaternion):
            self._buffer -= other.buffer
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __imul__(self, other: Union[float, int, 'Quaternion']) -> 'Quaternion':
        """
        Assign scale multiplication, q * scale.

        Args:
            other (Union[float, int, 'Quaternion']): other

        Returns:
            Quaternion: self *= other_quaternion
        """
        if isinstance(other, Quaternion):
            x, y, z, w = self.x, self.y, self.z, self.w
            self.x = w * other.x + x * other.w + y * other.z - z * other.y
            self.y = w * other.y + y * other.w + z * other.x - x * other.z
            self.z = w * other.z + z * other.w + x * other.y - y * other.x
            self.w = w * other.w - x * other.x - y * other.y - z * other.z
        elif isinstance(other, float) or isinstance(other, int):
            self._buffer *= other
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __idiv__(self, scale: Union[float, int]) -> 'Quaternion':
        """
        Assign a quaternion resulted by v / scale
        - division of a quaternion by a float r is scaling by (1/r)

        Args:
            scale (Union[float, int]): scale

        Returns:
            Quaternion: self /= other_quaternion
        """
        if isinstance(scale, float) or isinstance(scale, int):
            return self.__imul__(1.0 / scale)
        else:
            raise ValueError("Not supported type {}.".format(type(scale)))

    def __neg__(self) -> 'Quaternion':
        """
        Returns negated quaternion.
         - The negation of a quaternion is negation of all its coordinates

        Returns:
            Quaternion: the negated quaternion
        """
        return Quaternion(buffer=-self.buffer)

    def __iter__(self) -> Iterator[float]:
        """
        Iterator over the coordinates

        Returns:
            Iterator[float]: iterator
        """
        return self.buffer.__iter__()

    def __eq__(self, other: 'Quaternion') -> bool:
        """
        Equality of points is equality of all coordinates to within epsilon.

        Args:
            other (Quaternion): other to compare

        Returns:
            bool: True if the differences of all coordinates are within epsilon, Otherwise False.
        """
        return np.all(np.isclose(self.buffer, other.buffer))

    def __ne__(self, other: 'Quaternion') -> bool:
        """
        Inequality of points is inequality of any coordinates

        Args:
            other (Quaternion): other to compare

        Returns:
            bool: False if the differences of all coordinates are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __getitem__(self, i: int) -> float:
        """
        Return Q[i] where Q[i] is x, y, z, w for i in 0, 1, 2, 3 respectively.

        Args:
            i (int): index

        Returns:
            float: value at the given index.
        """
        return self.buffer[i]

    def __setitem__(self, i: int, value: float) -> None:
        """
        Set Q[i] where Q[i] is x, y, z, w for i in 0, 1, 2, 3 respectively.
        Args:
            i (int): index
            value (float): new value
        """
        self.buffer[i] = value

    def dot(self, other: 'Quaternion') -> float:
        """
        Returns the dot product.

        Args:
            other (Quaternion): other to dot product.

        Returns:
            float: the result of dot product
        """
        return float(np.dot(self.buffer, other.buffer))

    def _norm(self) -> np.ndarray:
        """
        Normalize this quaternion

        Returns:
            np.ndarray: unit quaternion
        """
        norm = np.linalg.norm(self._buffer)
        if norm == 0:
            return self._buffer
        unit_quaternion = self._buffer / norm
        return unit_quaternion

    def norm(self) -> 'Quaternion':
        """
        Returns the normalized quaternion

        Returns:
            Quaternion: unit quaternion
        """
        return Quaternion.from_numpy(self._norm())

    def normalize(self) -> 'Quaternion':
        """
        Normalize the quaternion in place

        Returns:
            Quaternion: normalized self (Quaternion) object
        """
        self._buffer = self._norm()
        return self

    def __str__(self) -> str:
        """
        String representation of a quaternion

        Returns:
            str: String representation of a quaternion
        """
        return "(%f,%f,%f,%f)" % (self.x, self.y, self.z, self.w)

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Quaternion" + str(self)
