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
"""A class for twist."""
from typing import Optional
from deepsim.core.vector3 import Vector3

from geometry_msgs.msg import Twist as ROSTwist


class Twist:
    """
    Twist class
    """
    def __init__(self,
                 linear: Optional[Vector3] = None,
                 angular: Optional[Vector3] = None) -> None:
        """
        Initialize Twist class

        Args:
            linear (Optional[Vector3]): linear velocity
            angular (Optional[Vector3]): angular velocity
        """
        self._linear = linear.copy() if linear else Vector3()
        self._angular = angular.copy() if angular else Vector3()

    @property
    def linear(self) -> Vector3:
        """
        Returns the copy of linear velocity

        Returns:
            Vector3: the copy of linear velocity
        """
        return self._linear.copy()

    @linear.setter
    def linear(self, value: Vector3) -> None:
        """
        Set the linear velocity

        Args:
            value (Vector3): the linear velocity
        """
        self._linear = value.copy()

    @property
    def angular(self) -> Vector3:
        """
        Returns the copy of angular velocity

        Returns:
            Vector3: the copy of angular velocity
        """
        return self._angular.copy()

    @angular.setter
    def angular(self, value: Vector3) -> None:
        """
        Set the angular velocity

        Args:
            value (Vector3): the angular velocity
        """
        self._angular = value.copy()

    def to_ros(self) -> ROSTwist:
        """
        Return the ROS Twist object created from this twist.

        Returns:
            gazebo_msgs.msg.Twist: ROS Twist
        """
        ros_pose = ROSTwist()
        if self.linear:
            ros_pose.linear = self.linear.to_ros()
        if self.angular:
            ros_pose.angular = self.angular.to_ros()
        return ros_pose

    @staticmethod
    def from_ros(value: ROSTwist) -> 'Twist':
        """
        Returns new Twist object created from ROS Twist

        Args:
            value (ROSTwist): ROS Twist

        Returns:
            Twist: new Twist object created from ROS Twist
        """
        return Twist(linear=Vector3.from_ros(value.linear),
                     angular=Vector3.from_ros(value.angular))

    def copy(self) -> 'Twist':
        """
        Returns a copy.

        Returns:
            Twist: the copied twist
        """
        return Twist(linear=self._linear,
                     angular=self._angular)

    def __add__(self, other: 'Twist') -> 'Twist':
        """
        Returns a twist sum.

        Args:
            other (Twist): other to add.

        Returns:
            Twist: a twist sum.
        """
        if isinstance(other, Twist):
            return Twist(linear=self._linear + other._linear,
                         angular=self._angular + other._angular)
        return NotImplemented

    def __sub__(self, other: 'Twist') -> 'Twist':
        """
        Returns a twist difference.

        Args:
            other (Twist): other to subtract.

        Returns:
            Twist: a twist difference.
        """
        if isinstance(other, Twist):
            return Twist(linear=self._linear - other._linear,
                         angular=self._angular - other._angular)
        return NotImplemented

    def __iadd__(self, other: 'Twist') -> 'Twist':
        """
        Assign the twist sum.

        Args:
            other (Twist): other to add.

        Returns:
            Twist: self += other
        """
        if isinstance(other, Twist):
            self._linear += other._linear
            self._angular += other._angular
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __isub__(self, other: 'Twist') -> 'Twist':
        """
        Assign the twist difference.

        Args:
            other (Twist): other to subtract.

        Returns:
            Twist: self -= other
        """
        if isinstance(other, Twist):
            self._linear -= other._linear
            self._angular -= other._angular
        else:
            raise ValueError("Not supported type {}.".format(type(other)))
        return self

    def __eq__(self, other: 'Twist') -> bool:
        """
        Equality of Twist.

        Args:
            other (Twist): other to compare

        Returns:
            bool: True if the differences of all components are within epsilon, Otherwise False.
        """
        return self._linear == other._linear and self._angular == other._angular

    def __ne__(self, other: 'Twist') -> bool:
        """
        Inequality of points is inequality of any coordinates

        Args:
            other (Twist): other to compare

        Returns:
            bool: False if the differences of all components are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __str__(self) -> str:
        """
        String representation of a twist

        Returns:
            str: String representation of a twist
        """
        return "(linear=%s, angular=%s)" % (repr(self._linear), repr(self._angular))

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Twist" + str(self)
