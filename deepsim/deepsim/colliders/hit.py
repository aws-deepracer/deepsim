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
"""A class for hit."""
from typing import Any
from deepsim.math.ray import Ray
from deepsim.math.vector3 import Vector3


class Hit:
    """
    Hit class
    """
    def __init__(self, obj: Any, ray: Ray, entry: float, exit: float,
                 entry_normal: Vector3 = None, exit_normal: Vector3 = None):
        """
        Initialize Hit class.

        Args:
            obj (Any): object
            ray (Ray): ray casted
            entry (float): entry distance from ray origin
            exit (float): exit distance from ray origin
            entry_normal (Vector3): entry surface normal
            exit_normal (Vector3): exit surface normal
        """
        self._obj = obj
        self._ray = ray
        self._entry = entry
        self._exit = exit
        self._entry_normal = entry_normal
        self._exit_normal = exit_normal

    @property
    def obj(self) -> Any:
        """
        Returns the object that hit by ray

        Returns:
            Any: the object that hit by ray
        """
        return self._obj

    @property
    def ray(self) -> Ray:
        """
        Returns the ray.

        Returns:
            Ray: the ray
        """
        return self._ray

    @property
    def entry(self) -> float:
        """
        Returns the entry distance from ray origin.

        Returns:
            float: the entry distance from ray origin.
        """
        return self._entry

    @property
    def exit(self) -> float:
        """
        Returns the exit distance from ray origin.

        Returns:
            float: the exit distance from ray origin.
        """
        return self._exit

    @property
    def entry_normal(self) -> Vector3:
        """
        Returns the surface normal at entry point.

        Returns:
            Vector3: the surface normal at entry point.
        """
        return self._entry_normal

    @property
    def exit_normal(self) -> Vector3:
        """
        Returns the surface normal at exit point.

        Returns:
            Vector3: the surface normal at exit point.
        """
        return self._exit_normal

    def __lt__(self, other: 'Hit') -> bool:
        """
        Compare Hit object with other.

        Args:
            other (Hit): the other hit object to compare

        Returns:
            bool: True if this hit's entry is less than other hit's entry.
        """
        return self.entry < other.entry

    def __gt__(self, other: 'Hit') -> bool:
        """
        Compare Hit object with other.

        Args:
            other (Hit): the other hit object to compare

        Returns:
            bool: True if this hit's entry is greater than other hit's entry.
        """
        return self.entry > other.entry

    def __repr__(self) -> str:
        """
        Representation

        Returns:
            str: string representation of Hit object.
        """
        return "Hit " + str(self.obj) + " Along " + repr(self.ray) + " entry: " + str(self.entry) \
               + " exit: " + str(self.exit)
