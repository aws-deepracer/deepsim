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
"""A class for model state."""
from typing import Optional
from deepsim.core.pose import Pose
from deepsim.core.twist import Twist

from gazebo_msgs.msg import ModelState as ROSModelState


class ModelState:
    """
    Model State class
    """

    def __init__(self,
                 model_name: Optional[str] = None,
                 pose: Optional[Pose] = None,
                 twist: Optional[Twist] = None,
                 reference_frame: Optional[str] = None):
        """
        Initialize ModelState class

        Args:
            model_name (Optional[str]): model name
            pose (Optional[Pose]): desired pose in reference frame
            twist (Optional[Twist]): desired twist in reference frame
            reference_frame (Optional[str]): set pose/twist relative to the frame of this entity (Body/Model)
                                   leave empty or "world" or "map" defaults to world-frame
        """
        self._model_name = model_name
        self._pose = pose.copy() if pose else Pose()
        self._twist = twist.copy() if twist else Twist()
        self._reference_frame = reference_frame or ''

    @property
    def model_name(self) -> str:
        """
        Returns the model name

        Returns:
            str: model name
        """
        return self._model_name

    @model_name.setter
    def model_name(self, value: str) -> None:
        """
        Set model name

        Args:
            value (str): model name
        """
        self._model_name = value

    @property
    def pose(self) -> Pose:
        """
        Returns the copy of pose.

        Returns:
            Pose: the copy of pose of the model
        """
        return self._pose.copy()

    @pose.setter
    def pose(self, value: Pose) -> None:
        """
        Set the pose.

        Args:
            value (Pose): the pose
        """
        self._pose = value.copy()

    @property
    def twist(self) -> Twist:
        """
        Return the copy of twist.

        Returns:
            Twist: the copy of twist
        """
        return self._twist.copy()

    @twist.setter
    def twist(self, value: Twist) -> None:
        """
        Set the twist.

        Args:
            value (Twist): the twist
        """
        self._twist = value.copy()

    @property
    def reference_frame(self) -> str:
        """
        Returns the reference frame

        Returns:
            str: the reference frame
        """
        return self._reference_frame

    @reference_frame.setter
    def reference_frame(self, value: str) -> None:
        """
        Set the reference frame

        Args:
            value (str): the reference frame
        """
        self._reference_frame = value

    def to_ros(self) -> ROSModelState:
        """
        Return the ROS ModelState object created from this model state.

        Returns:
            gazebo_msgs.msg.ModelState: ROS ModelState
        """
        ros_model_state = ROSModelState()
        if self.model_name:
            ros_model_state.model_name = self.model_name
        if self._pose:
            ros_model_state.pose = self._pose.to_ros()
        if self._twist:
            ros_model_state.twist = self._twist.to_ros()
        if self.reference_frame:
            ros_model_state.reference_frame = self.reference_frame
        return ros_model_state

    @staticmethod
    def from_ros(value: ROSModelState) -> 'ModelState':
        """
        Returns new ModelState object created from ROS ModelState

        Args:
            value (ROSModelState): ROS ModelState

        Returns:
            ModelState: new ModelState object created from ROS LinkState
        """
        return ModelState(model_name=value.model_name,
                          pose=Pose.from_ros(value.pose),
                          twist=Twist.from_ros(value.twist),
                          reference_frame=value.reference_frame)

    def copy(self) -> 'ModelState':
        """
        Returns a copy.

        Returns:
            ModelState: the copied model state
        """
        return ModelState(model_name=self.model_name,
                          pose=self._pose,
                          twist=self._twist,
                          reference_frame=self.reference_frame)

    def __eq__(self, other: 'ModelState') -> bool:
        """
        Equality of ModelState.

        Args:
            other (ModelState): other to compare

        Returns:
            bool: True if the differences of all components are within epsilon, Otherwise False.
        """
        return (self.model_name == other.model_name and self.reference_frame == other.reference_frame
                and self._pose == other._pose and self._twist == other._twist)

    def __ne__(self, other: 'ModelState') -> bool:
        """
        Inequality of points is inequality of any coordinates

        Args:
            other (ModelState): other to compare

        Returns:
            bool: False if the differences of all components are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __str__(self) -> str:
        """
        String representation of a model state

        Returns:
            str: String representation of a model state
        """
        return "(model_name=%s, pose=%s, twist=%s, reference_frame=%s)" % (self.model_name,
                                                                           repr(self._pose),
                                                                           repr(self._twist),
                                                                           self.reference_frame)

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "ModelState" + str(self)
