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
"""A class for pose."""
from typing import Optional, Union
from deepsim.math.point import Point
from deepsim.math.quaternion import Quaternion
from deepsim.math.vector3 import Vector3

from geometry_msgs.msg import Pose as ROSPose


class Pose:
    """
    Pose class
    """
    def __init__(self, position: Optional[Union[Point, Vector3]] = None, orientation: Optional[Quaternion] = None) -> None:
        """
        Initialize Pose class

        Args:
            position (Optional[Union[Point, Vector3]]): position
            orientation (Optional[Quaternion]): quaternion
        """
        if isinstance(position, Vector3):
            position = position.to_point()
        self._position = position.copy() if position else Point()
        self._orientation = orientation.copy() if orientation else Quaternion()

    @property
    def position(self) -> Point:
        """
        Returns the copy of position

        Returns:
            Point: the copy of position
        """
        return self._position.copy()

    @position.setter
    def position(self, value: Point) -> None:
        """
        Set the position

        Args:
            value (Point): the position
        """
        self._position = value.copy()

    @property
    def orientation(self) -> Quaternion:
        """
        Returns the copy of orientation

        Returns:
            Quaternion: the copy of orientation
        """
        return self._orientation.copy()

    @orientation.setter
    def orientation(self, value: Quaternion) -> None:
        """
        Set the orientation

        Args:
            value (Quaternion): the orientation
        """
        self._orientation = value.copy()

    def to_ros(self) -> ROSPose:
        """
        Return the ROS Point object created from this pose.

        Returns:
            gazebo_msgs.msg.Pose: ROS Pose
        """
        ros_pose = ROSPose()
        if self._position:
            ros_pose.position = self._position.to_ros()
        if self._orientation:
            ros_pose.orientation = self._orientation.to_ros()
        return ros_pose

    @staticmethod
    def from_ros(value: ROSPose) -> 'Pose':
        """
        Returns new Pose object created from ROS Pose

        Args:
            value (geometry_msgs.msg.Pose): ROS Pose

        Returns:
            Pose: new Pose object created from ROS Pose
        """
        return Pose(position=Point.from_ros(value.position),
                    orientation=Quaternion.from_ros(value.orientation))

    def copy(self) -> 'Pose':
        """
        Returns a copy.

        Returns:
            Pose: the copied pose
        """
        return Pose(position=self._position,
                    orientation=self._orientation)

    def __add__(self, other: 'Pose') -> 'Pose':
        """
        Returns a pose sum.

        Args:
            other (Pose): other to add.

        Returns:
            Pose: a pose sum.
        """
        if isinstance(other, Pose):
            return Pose(position=self._position + other._position.to_vector().rotate(self._orientation),
                        orientation=self._orientation * other._orientation)
        return NotImplemented

    def __sub__(self, other: 'Pose') -> 'Pose':
        """
        Returns a pose difference.

        Args:
            other (Pose): other to subtract.

        Returns:
            Pose: a pose difference.
        """
        if isinstance(other, Pose):
            return Pose(position=self._position - other._position.to_vector().rotate(self._orientation),
                        orientation=self._orientation * other._orientation.inverse())
        return NotImplemented

    def __iadd__(self, other: 'Pose') -> 'Pose':
        """
        Assign the pose sum.

        Args:
            other (Pose): other to add.

        Returns:
            Pose: self += other_pose
        """
        if isinstance(other, Pose):
            self._position += other._position.to_vector().rotate(self._orientation)
            self._orientation *= other._orientation
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __isub__(self, other: 'Pose') -> 'Pose':
        """
        Assign the pose difference.

        Args:
            other (Pose): other to subtract.

        Returns:
            Pose: self += other_pose
        """
        if isinstance(other, Pose):
            self._position -= other._position.to_vector().rotate(self._orientation)
            self._orientation *= other._orientation.inverse()
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __neg__(self) -> 'Pose':
        """
        Returns negated pose.
         - The negation of a vector is negation of all its coordinates

        Returns:
            Vector3: the negated vector
        """
        return Pose(-self._position, -self._orientation)

    def __eq__(self, other: 'Pose') -> bool:
        """
        Equality of poses is equality of position and orientation to within epsilon.

        Args:
            other (Pose): other to compare

        Returns:
            bool: True if the differences of all components are within epsilon, Otherwise False.
        """
        return self._position == other._position and self._orientation == other._orientation

    def __ne__(self, other: 'Pose') -> bool:
        """
        Inequality of pose is inequality of any coordinates

        Args:
            other (Pose): other to compare

        Returns:
            bool: False if the differences of all coordinates are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __str__(self) -> str:
        """
        String representation of a pose

        Returns:
            str: String representation of a pose
        """
        return "(position=%s, orientation=%s)" % (repr(self._position), repr(self._orientation))

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Pose" + str(self)
