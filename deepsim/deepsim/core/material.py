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
"""A class for material."""
from typing import Optional
from deepsim.core.color import Color


class Material:
    """
    Material class
    """
    def __init__(self,
                 ambient: Optional[Color] = None,
                 diffuse: Optional[Color] = None,
                 specular: Optional[Color] = None,
                 emissive: Optional[Color] = None) -> None:
        """
        Initialize Material class

        Args:
            ambient (Optional[Color]): ambient color
            diffuse (Optional[Color]):  diffuse color
            specular (Optional[Color]):  specular color
            emissive (Optional[Color]): emissive color
        """
        self._ambient = ambient.copy() if ambient else Color()
        self._diffuse = diffuse.copy() if diffuse else Color()
        self._specular = specular.copy() if specular else Color()
        self._emissive = emissive.copy() if emissive else Color()

    @property
    def ambient(self) -> Color:
        """
        Returns the copy of ambient color

        Returns:
            Color: the copy of ambient color of the material
        """
        return self._ambient.copy()

    @ambient.setter
    def ambient(self, value: Color) -> None:
        """
        Set ambient color

        Args:
            value (Color): ambient color
        """
        self._ambient = value.copy()

    @property
    def diffuse(self) -> Color:
        """
        Returns the copy of diffuse color

        Returns:
            Color: the copy of diffuse color of the material
        """
        return self._diffuse.copy()

    @diffuse.setter
    def diffuse(self, value: Color) -> None:
        """
        Set diffuse color

        Args:
            value (Color): diffuse color
        """
        self._diffuse = value.copy()

    @property
    def specular(self) -> Color:
        """
        Returns the copy of specular color

        Returns:
            Color: the copy of specular color of the material
        """
        return self._specular.copy()

    @specular.setter
    def specular(self, value: Color) -> None:
        """
        Set specular color

        Args:
            value (Color): specular color
        """
        self._specular = value.copy()

    @property
    def emissive(self) -> Color:
        """
        Returns the copy of emissive color

        Returns:
            Color: the copy of emissive color of the material
        """
        return self._emissive.copy()

    @emissive.setter
    def emissive(self, value: Color) -> None:
        """
        Set emissive color

        Args:
            value (Color): emissive color
        """
        self._emissive = value.copy()

    def copy(self) -> 'Material':
        """
        Returns a copy.

        Returns:
            Material: the copied material
        """
        return Material(ambient=self._ambient,
                        diffuse=self._diffuse,
                        specular=self._specular,
                        emissive=self._emissive)

    def __eq__(self, other: 'Material') -> bool:
        """
        Equality of Material.

        Args:
            other (Material): other to compare

        Returns:
            bool: True if the differences of all components are within epsilon, Otherwise False.
        """
        return (self._ambient == other._ambient and self._diffuse == other._diffuse
                and self._specular == other._specular and self._emissive == other._emissive)

    def __ne__(self, other: 'Material') -> bool:
        """
        Inequality of points is inequality of any coordinates

        Args:
            other (Material): other to compare

        Returns:
            bool: False if the differences of all components are within epsilon, Otherwise True.
        """
        return not self.__eq__(other)

    def __str__(self) -> str:
        """
        String representation of a link state

        Returns:
            str: String representation of a link state
        """
        return "(ambient=%s, diffuse=%s, specular=%s, emissive=%s)" % (repr(self._ambient),
                                                                       repr(self._diffuse),
                                                                       repr(self._specular),
                                                                       repr(self._emissive))

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Material" + str(self)
