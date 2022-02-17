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
"""A class for abstract collider."""
import abc
from enum import Enum
from typing import Optional, Union
from threading import RLock

from deepsim.behaviours.transform import Transform
from deepsim.core.vector3 import Vector3
from deepsim.core.point import Point
from deepsim.core.pose import Pose
from deepsim.colliders.ray_castable import RayCastable

from shapely.geometry.base import BaseGeometry

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class ColliderType(Enum):
    """
    Collider Type
    """
    COLLIDER_2D = 0
    COLLIDER_3D = 1


class AbstractCollider(RayCastable, metaclass=abc.ABCMeta):
    """
    Abstract Collider class
    """
    def __init__(self,
                 collider_type: ColliderType,
                 pose_offset: Optional[Pose] = None,
                 transform: Optional[Transform] = None,
                 enable_on_init: bool = True):
        """
        Initialize collider.

        Args:
            collider_type (ColliderType): collider type (2d vs 3d)
            pose_offset (Optional[Pose]): pose offset.
            transform (Optional[Transform]): transform to be tracked by the collider.
            enable_on_init: (bool): the flag whether to enable the collider on initialize.
        """
        self._type = collider_type
        self._transform = transform
        self._pose_offset = pose_offset.copy() if pose_offset else Pose()
        self._transform_lock = RLock()
        self._is_enabled = False
        if enable_on_init:
            self.enable()

    @property
    def is_enabled(self) -> bool:
        """
        Returns whether this collider is enabled or not.

        Returns:
            bool: the flag whether this collider is enabled or not.
        """
        return self._is_enabled

    def enable(self) -> None:
        """
        Enable the collider
        """
        self._is_enabled = True

    def disable(self) -> None:
        """
        Disable the collider.
        """
        self._is_enabled = False

    def attach(self, transform: Transform) -> None:
        """
        Attach the transform to track

        Args:
            transform [Transform]: the transform to be tracked.
        """
        with self._transform_lock:
            self.on_attach()
            self._transform = transform

    def detach(self) -> None:
        """
        Detach the transform.
        - collider goes back to the internal representation.
        """
        with self._transform_lock:
            try:
                self.on_detach()
            finally:
                self._transform = None

    def validate(self, target: Union['AbstractCollider', Vector3, Point]) -> None:
        """
        Validate if input is compatible with collider.
        - raises ValueError if incompatible.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check the compatibility.
        """
        if isinstance(target, AbstractCollider):
            if target.type != self.type:
                raise ValueError("Incompatible type given: {}({})".format(type(target),
                                                                          target.type.name))

    @property
    def pose_offset(self) -> Pose:
        """
        Returns a copy of pose offset.

        Returns:
            Pose: a copy of pose offset.
        """
        return self._pose_offset.copy()

    @pose_offset.setter
    def pose_offset(self, value: Pose) -> None:
        """
        Set pose offset with a copy of given pose.

        Args:
            value (Pose): new pose
        """
        self._pose_offset = value.copy()

    @property
    def type(self) -> ColliderType:
        """
        Returns collider type.

        Returns:
            ColliderType: the type of collider (2d vs 3d).
        """
        return self._type

    @property
    def transform(self) -> Transform:
        """
        Returns the transform tracking.

        Returns:
            Transform: the transform that the collider is tracking.
        """
        with self._transform_lock:
            return self._transform

    def intersects(self, target: Union['AbstractCollider', Vector3, Point]) -> bool:
        """
        Check if the collider intersects with target.
        - if either self or target is not enabled then returns False

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether intersecting

        Returns:
            bool: True if intersects otherwise False.
        """
        self.validate(target=target)
        if not self.is_enabled:
            return False
        if isinstance(target, AbstractCollider) and not target.is_enabled:
            return False
        return self._intersects(target=target)

    def contains(self, target: Union['AbstractCollider', Vector3, Point]) -> bool:
        """
        Check if the collider contains with target.
        - if either self or target is not enabled then returns False

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether collider contains.

        Returns:
            bool: True if collider contains target otherwise False.
        """
        self.validate(target=target)
        if not self.is_enabled:
            return False
        if isinstance(target, AbstractCollider) and not target.is_enabled:
            return False
        return self._contains(target=target)

    def __contains__(self, item: Union['AbstractCollider', Vector3, Point]) -> bool:
        """
        Check if the collider contains with target.
        - if either self or target is not enabled then returns False

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether collider contains.

        Returns:
            bool: True if collider contains target otherwise False.
        """
        self.validate(target=item)
        if not self.is_enabled:
            return False
        if isinstance(item, AbstractCollider) and not item.is_enabled:
            return False
        return self.contains(target=item)

    def on_attach(self) -> None:
        """
        Callback on attach (right before attach)
        """
        pass

    def on_detach(self):
        """
        Callback on detach (right before detach)
        """
        pass

    @abc.abstractmethod
    def _intersects(self, target: Union['AbstractCollider', Vector3, Point]) -> bool:
        """
        Check if the collider intersects with target.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether intersecting

        Returns:
            bool: True if intersects otherwise False.
        """
        raise NotImplementedError("Collider must implement intersects!")

    @abc.abstractmethod
    def _contains(self, target: Union['AbstractCollider', Vector3, Point]) -> bool:
        """
        Check if the collider contains with target.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether collider contains.

        Returns:
            bool: True if collider contains target otherwise False.
        """
        raise NotImplementedError("Collider must implement contains!")


class Abstract2DCollider(AbstractCollider, metaclass=abc.ABCMeta):
    """
    Abstract2DCollider class
    """
    def __init__(self, transform: Optional[Transform] = None, *,
                 pose: Optional[Pose] = None,
                 pose_offset: Optional[Pose] = None,
                 enable_on_init: bool = True) -> None:
        """
        Initialize Abstract2DCollider class

        Args:
            transform (Optional[Transform]): transform to be tracked by the collider.
            pose: (Optional[Pose]): pose of the collider.
            pose_offset (Optional[Pose]): pose offset.
            enable_on_init: (bool): the flag whether to enable the collider on initialize.
        """
        AbstractCollider.__init__(self,
                                  collider_type=ColliderType.COLLIDER_2D,
                                  pose_offset=pose_offset,
                                  transform=transform,
                                  enable_on_init=enable_on_init)
        self._pose = pose.copy() if pose else Pose()

    @property
    def pose(self) -> Pose:
        """
        Returns the copy of pose.
        - If tracking transform then it will return transform pose, otherwise will return the copy of internal pose.

        Returns:
            Pose: transform pose if tracking transform otherwise the copy of internal pose.
        """
        transform = self.transform
        if transform:
            return transform.state.pose
        return self._pose.copy()

    @pose.setter
    def pose(self, value: Pose) -> None:
        """
        Set pose with given pose.
        - This won't have any impact during tracking the transform.

        Args:
            value (Pose): new pose
        """
        self._pose = value.copy()

    @property
    def adjusted_pose(self) -> Pose:
        """
        Returns the adjusted pose (pose + pose_offset).

        Returns:
            Pose: the adjusted pose (pose + pose_offset).
        """
        return self.pose + self._pose_offset

    def on_detach(self) -> None:
        """
        Preserve last transform pose prior to detach.
        """
        transform = self.transform
        if transform:
            self._pose = transform.state.pose

    def _intersects(self, target: Union[AbstractCollider, Vector3, Point]) -> bool:
        """
        Check if the collider intersects with target.
        - Test ignores z-axis, so even two 2d polygon doesn't intersect with respect to z-axis,
          if their x and y coordinates intersect then the test returns True.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether intersecting

        Returns:
            bool: True if intersects otherwise False.
        """
        return self.to_shapely().intersects(target.to_shapely())

    def _contains(self, target: Union[AbstractCollider, Vector3, Point]) -> bool:
        """
        Check if the collider contains with target.
        - Test ignores z-axis, so even two 2d polygon doesn't intersect with respect to z-axis,
          if other plane's x and y coordinates are within this polygon then the test returns True.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether collider contains.

        Returns:
            bool: True if collider contains target otherwise False.
        """
        return self.to_shapely().contains(target.to_shapely())

    @abc.abstractmethod
    def to_shapely(self) -> BaseGeometry:
        """
        Returns shapely geometry

        Returns:
            BaseGeometry: shapely geometry representing the collider.
        """
        raise NotImplementedError("Subclass must implement this!")


class Abstract3DCollider(AbstractCollider, metaclass=abc.ABCMeta):
    """
    Abstract3DCollider class
    """
    def __init__(self,
                 transform: Optional[Transform] = None, *,
                 pose_offset: Optional[Pose] = None,
                 enable_on_init: bool = True):
        """
        Initialize Abstract3DCollider class

        Args:
            transform (Optional[Transform]): transform to be tracked by the collider.
            pose_offset (Optional[Pose]): pose offset.
            enable_on_init: (bool): the flag whether to enable the collider on initialize.
        """
        AbstractCollider.__init__(self,
                                  collider_type=ColliderType.COLLIDER_3D,
                                  pose_offset=pose_offset,
                                  transform=transform,
                                  enable_on_init=enable_on_init)
