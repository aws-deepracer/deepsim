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
"""A class for color."""
from typing import Union, Iterable, Optional, Iterator
import numpy as np
from std_msgs.msg import ColorRGBA


class Color:
    """
    Color class
    """
    def __init__(self, r: float = 0.0, g: float = 0.0, b: float = 0.0, a: float = 1.0, *,
                 buffer: Optional[Iterable[float]] = None):
        """
        Initialize Color class

        Args:
            r (float): red value
            g (float): green value
            b (float): blue value
            a (float): alpha value
            buffer (Optional[Iterable[float]]): buffer to copy directly to internal representation.
        """
        buffer = buffer if buffer is not None else [r, g, b, a]
        if len(buffer) < 4:
            raise ValueError("buffer must contain at least 4 elements")
        self._buffer = np.array(buffer[:4], dtype=float)

    @property
    def r(self) -> float:
        """
        Returns red value of the color.

        Returns:
            float: red value of the color.
        """
        return self._buffer[0]

    @r.setter
    def r(self, value: float) -> None:
        """
        Set red value of the color

        Args:
            value (float): new red value of the color
        """
        self._buffer[0] = value

    @property
    def g(self) -> float:
        """
        Returns green value of the color.

        Returns:
            float: green value of the color.
        """
        return self._buffer[1]

    @g.setter
    def g(self, value: float) -> None:
        """
        Set green value of the color

        Args:
            value (float): new green value of the color
        """
        self._buffer[1] = value

    @property
    def b(self) -> float:
        """
        Returns blue value of the color.

        Returns:
            float: blue value of the color.
        """
        return self._buffer[2]

    @b.setter
    def b(self, value: float) -> None:
        """
        Set blue value of the color

        Args:
            value (float): new blue value of the color
        """
        self._buffer[2] = value

    @property
    def a(self) -> float:
        """
        Returns alpha value of the color.

        Returns:
            float: alpha value of the color.
        """
        return self._buffer[3]

    @a.setter
    def a(self, value: float) -> None:
        """
        Set alpha value of the color

        Args:
            value (float): new alpha value of the color
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

    def to_ros(self) -> ColorRGBA:
        """
        Convert to ROS model

        Returns:
            std_msgs.msg.ColorRGBA: ROS ColorRGBA
        """
        ros_color = ColorRGBA()
        ros_color.r = self.r
        ros_color.g = self.g
        ros_color.b = self.b
        ros_color.a = self.a
        return ros_color

    def to_list(self) -> list:
        """
        Convert to Python list

        Returns:
            list: list containing r, g, b, a in order.
        """
        return list(self._buffer)

    def to_numpy(self) -> np.ndarray:
        """
        Convert to Numpy array

        Returns:
            numpy.array: numpy array containing r, g, b, a in order.
        """
        return self._buffer.copy()

    @staticmethod
    def from_ros(value: ColorRGBA) -> 'Color':
        """
        Return new Color object from ROS ColorRGBA

        Args:
            value (std_msgs.msg.ColorRGBA): ROS ColorRGBA

        Returns:
            Color: new Color object created from ROS ColorRGBA
        """
        return Color(r=value.r,
                     g=value.g,
                     b=value.b,
                     a=value.a)

    @staticmethod
    def from_list(value: Union[list, tuple]) -> 'Color':
        """
        Return new Color object from list

        Args:
            value (Union[list, tuple]): Color in list or tuple type

        Returns:
            Color: new Color object created from list
        """
        return Color(buffer=value)

    @staticmethod
    def from_numpy(value: np.ndarray) -> 'Color':
        """
        Return new Color object from numpy.array

        Args:
            value (np.ndarray): Color in numpy.array type

        Returns:
            Color: new Color object created from numpy.array
        """
        return Color(buffer=value)

    def copy(self) -> 'Color':
        """
        Returns a copy.

        Returns:
            Color: the copied color
        """
        return Color(buffer=self.buffer)

    def __iter__(self) -> Iterator[float]:
        """
        Iterator over the coordinates

        Returns:
            Iterator[float]: iterator
        """
        return self.buffer.__iter__()

    def __eq__(self, other: 'Color') -> bool:
        """
        Equality of points is equality of all color values to within epsilon.

        Args:
            other (Color): other to compare

        Returns:
            bool: True if the differences of all coordinates are within epsilon, Otherwise False.
        """
        return np.all(np.isclose(self.buffer, other.buffer))

    def __ne__(self, other: 'Color') -> bool:
        """
        Inequality of points is inequality of any coordinates

        Args:
            other (Color): other to compare

        Returns:
            bool: False if the differences of all coordinates are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __getitem__(self, i: int) -> float:
        """
        Return C[i] where C[i] is r, g, b, a for i in 0, 1, 2, 3 respectively.

        Args:
            i (int): index

        Returns:
            float: value at the given index.
        """
        return self.buffer[i]

    def __setitem__(self, i: int, value: float) -> None:
        """
        Set C[i] where C[i] is r, g, b, a for i in 0, 1, 2, 3 respectively.
        Args:
            i (int): index
            value (float): new value
        """
        self.buffer[i] = value

    def __str__(self) -> str:
        """
        String representation of a color

        Returns:
            str: String representation of a color
        """
        return "(%f,%f,%f,%f)" % (self.r, self.g, self.b, self.a)

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Color" + str(self)
