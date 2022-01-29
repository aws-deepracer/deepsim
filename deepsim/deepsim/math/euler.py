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
"""A class for euler."""
from typing import TypeVar, Union, Iterable, Optional, Iterator
import numpy as np
from deepsim.math.math import euler_to_quaternion, quaternion_to_euler
Quaternion = TypeVar('Quaternion')


class Euler:
    """
    Euler class
    """
    def __init__(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, *,
                 buffer: Optional[Iterable[float]] = None):
        """
        Initialize Euler class

        Args:
            roll (float): roll angle in radian (x-axis)
            pitch (float): pitch angle in radian (y-axis)
            yaw (float): yaw angle in radian (z-axis)
            buffer (Optional[Iterable[float]]): buffer to copy directly to internal representation.
        """
        buffer = buffer if buffer is not None else [roll, pitch, yaw]
        if len(buffer) < 3:
            raise ValueError("buffer must contain at least 3 elements")
        self._buffer = np.array(buffer[:3], dtype=float)

    @property
    def roll(self) -> float:
        """
        Returns roll angle in radian of euler.

        Returns:
            float: roll angle in radian of euler.
        """
        return self._buffer[0]

    @roll.setter
    def roll(self, value: float) -> None:
        """
        Set the roll angle in radian with given value.

        Args:
            value (float): new roll angle in radian
        """
        self._buffer[0] = value

    @property
    def pitch(self) -> float:
        """
        Returns pitch angle in radian of euler.

        Returns:
            float: pitch angle in radian of euler.
        """
        return self._buffer[1]

    @pitch.setter
    def pitch(self, value: float) -> None:
        """
        Set the pitch angle in radian with given value.

        Args:
            value (float): new pitch angle in radian
        """
        self._buffer[1] = value

    @property
    def yaw(self) -> float:
        """
        Returns yaw angle in radian of euler.

        Returns:
            float: yaw angle in radian of euler.
        """
        return self._buffer[2]

    @yaw.setter
    def yaw(self, value: float) -> None:
        """
        Set the yaw angle in radian with given value.

        Args:
            value (float): new yaw angle in radian
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
    def identity() -> 'Euler':
        """
        Returns identity in Euler.

        Returns:
            Euler: identity.
        """
        return Euler(0.0, 0.0, 0.0)

    def to_list(self) -> list:
        """
        Convert to Python list

        Returns:
            list: list containing roll, pitch, yaw in order.
        """
        return list(self._buffer)

    def to_numpy(self) -> np.ndarray:
        """
        Convert to Numpy array

        Returns:
            numpy.array: numpy array containing roll, pitch, yaw in order.
        """
        return self._buffer.copy()

    def to_quaternion(self) -> Quaternion:
        """
        Returns new Quaternion object converted from this Euler.

        Returns:
            Quaternion: new Quaternion object converted from Euler.
        """
        from deepsim.math.quaternion import Quaternion
        q = euler_to_quaternion(roll=self.roll, pitch=self.pitch, yaw=self.yaw)
        return Quaternion(*q)

    @staticmethod
    def from_list(value: Union[list, tuple]) -> 'Euler':
        """
        Return new Euler object from list.

        Args:
            value (Union[list, tuple]): euler in list or tuple type

        Returns:
            Euler: new Euler object created from list
        """
        return Euler(buffer=value)

    @staticmethod
    def from_numpy(value: np.ndarray) -> 'Euler':
        """
        Return new Euler object from numpy.array.

        Args:
            value (np.ndarray): euler in np.ndarray

        Returns:
            Euler: new Euler object created from numpy.array
        """
        return Euler(buffer=value)

    @staticmethod
    def from_quaternion(value: Union[Quaternion, list, tuple, np.ndarray]) -> 'Euler':
        """
        Return new Euler object from quaternion.

        Args:
            value (Union[Quaternion, list, tuple, np.ndarray]): quaternion

        Returns:
            Euler: new Euler object created from quaternion
        """
        from deepsim.math.quaternion import Quaternion
        if isinstance(value, list) or isinstance(value, tuple):
            value = Quaternion.from_list(value)
        elif type(value).__module__ == np.__name__:
            value = Quaternion.from_numpy(value)
        euler = quaternion_to_euler(x=value.x, y=value.y, z=value.z, w=value.w)
        return Euler(*euler)

    def copy(self) -> 'Euler':
        """
        Returns a copy.

        Returns:
            Euler: the copied euler
        """
        return Euler(buffer=self.buffer)

    def __iter__(self) -> Iterator[float]:
        """
        Iterator over the coordinates

        Returns:
            Iterator[float]: iterator
        """
        return self.buffer.__iter__()

    def __eq__(self, other: 'Euler') -> bool:
        """
        Equality of points is equality of all euler values to within epsilon.

        Args:
            other (Euler): other to compare

        Returns:
            bool: True if the differences of all coordinates are within epsilon, Otherwise False.
        """
        return np.all(np.isclose(self.buffer, other.buffer))

    def __ne__(self, other: 'Euler') -> bool:
        """
        Inequality of points is inequality of any coordinates

        Args:
            other (Euler): other to compare

        Returns:
            bool: False if the differences of all coordinates are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __getitem__(self, i: int) -> float:
        """
        Return E[i] where E[i] is roll, pitch, yaw for i in 0, 1, 2 respectively.

        Args:
            i (int): index

        Returns:
            float: value at the given index.
        """
        return self.buffer[i]

    def __setitem__(self, i: int, value: float) -> None:
        """
        Set E[i] where E[i] is roll, pitch, yaw for i in 0, 1, 2 respectively.
        Args:
            i (int): index
            value (float): new value
        """
        self.buffer[i] = value

    def __str__(self) -> str:
        """
        String representation of a euler

        Returns:
            str: String representation of a euler
        """
        return "(%f,%f,%f)" % (self.roll, self.pitch, self.yaw)

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Euler" + str(self)
