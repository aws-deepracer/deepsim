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
"""A class for light randomizer."""
import numpy as np

from deepsim.domain_randomizations.abs_randomizer import AbstractRandomizer
from deepsim.domain_randomizations.constants import (RangeType,
                                                     RANGE_MIN, RANGE_MAX,
                                                     ColorAttr, Attenuation)
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper

from std_msgs.msg import ColorRGBA
from gazebo_msgs.srv import SetLightProperties, SetLightPropertiesRequest


class LightRandomizer(AbstractRandomizer):
    """Light Randomizer class"""
    def __init__(self, light_name: str, color_range: dict = None, attenuation_range: dict = None):
        """
        Constructor

        Args:
            light_name (str): name of the light
            color_range (dict): min-max of each color component (r, g, b).
                Valid format: {'r': {'min': 0.0, 'max': 1.0},
                               'g': {'min': 0.0, 'max': 1.0},
                               'b': {'min': 0.0, 'max': 1.0}}
            attenuation_range (dict): min-max of each attenuation component (constant, linear, quadratic).
                Valid format: {'constant': {'min': 0.0, 'max':1.0},
                               'linear': {'min': 0.0, 'max':1.0},
                               'quadratic': {'min': 0.0, 'max':1.0}}
        """
        super(LightRandomizer, self).__init__()
        self._light_name = light_name

        self._range = {RangeType.COLOR: {ColorAttr.R: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                                         ColorAttr.G: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                                         ColorAttr.B: {RANGE_MIN: 0.0, RANGE_MAX: 1.0}},
                       RangeType.ATTENUATION: {Attenuation.CONSTANT: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                                               Attenuation.LINEAR: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                                               Attenuation.QUADRATIC: {RANGE_MIN: 0.0, RANGE_MAX: 1.0}}}
        if color_range:
            self._range[RangeType.COLOR].update(color_range)
        if attenuation_range:
            self._range[RangeType.ATTENUATION].update(attenuation_range)

        # ROS Services
        self._set_light_prop = ServiceProxyWrapper(GazeboServiceName.SET_LIGHT_PROPERTIES, SetLightProperties)

    @property
    def light_name(self) -> str:
        """
        Returns the name of light.

        Returns:
            str: the name of light.
        """
        return self._light_name

    @light_name.setter
    def light_name(self, value: str) -> None:
        """
        Set the light name.

        Args:
            value (str): the light name
        """
        self._light_name = value

    @property
    def color_range(self) -> dict:
        """
        Returns the color range configuration in dict.

        Returns:
            dict: the color range configuration in dict.
        """
        return self._range[RangeType.COLOR].copy()

    @color_range.setter
    def color_range(self, value: dict) -> None:
        """
        Set the color range configuration given.

        Args:
            value (dict): the color range configuration given.
        """
        self._range[RangeType.COLOR].update(value)

    @property
    def attenuation_range(self) -> dict:
        """
        Returns the attenuation range configuration in dict.

        Returns:
            dict: the attenuation range configuration in dict.
        """
        return self._range[RangeType.ATTENUATION].copy()

    @attenuation_range.setter
    def attenuation_range(self, value: dict) -> None:
        """
        Set the attenuation range configuration given.

        Args:
            value (dict): the attenuation range configuration given.
        """
        self._range[RangeType.ATTENUATION].update(value)

    def _randomize(self) -> None:
        """
        Randomize
        """
        req = SetLightPropertiesRequest()
        req.light_name = self._light_name

        color_range = self._range[RangeType.COLOR]
        req.diffuse = ColorRGBA(*[np.random.uniform(color_range[ColorAttr.R][RANGE_MIN],
                                                    color_range[ColorAttr.R][RANGE_MAX]),
                                  np.random.uniform(color_range[ColorAttr.G][RANGE_MIN],
                                                    color_range[ColorAttr.G][RANGE_MAX]),
                                  np.random.uniform(color_range[ColorAttr.B][RANGE_MIN],
                                                    color_range[ColorAttr.B][RANGE_MAX]),
                                  1.0])

        attenuation_range = self._range[RangeType.ATTENUATION]
        req.attenuation_constant = np.random.uniform(attenuation_range[Attenuation.CONSTANT][RANGE_MIN],
                                                     attenuation_range[Attenuation.CONSTANT][RANGE_MAX])
        req.attenuation_linear = np.random.uniform(attenuation_range[Attenuation.LINEAR][RANGE_MIN],
                                                   attenuation_range[Attenuation.LINEAR][RANGE_MAX])
        req.attenuation_quadratic = np.random.uniform(attenuation_range[Attenuation.QUADRATIC][RANGE_MIN],
                                                      attenuation_range[Attenuation.QUADRATIC][RANGE_MAX])
        self._set_light_prop(req)
