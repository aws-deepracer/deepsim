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
from typing import Any, Callable
from unittest import TestCase
from unittest.mock import patch, MagicMock, call
import inspect

from deepsim.gazebo.constants import GazeboServiceName
from deepsim.domain_randomizations.randomizers.light_randomizer import LightRandomizer

from gazebo_msgs.srv import SetLightProperties


@patch("deepsim.domain_randomizations.randomizers.light_randomizer.ServiceProxyWrapper")
class LightRandomizerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, service_proxy_wrapper_mock):
        light_name = "test_light"
        LightRandomizer(light_name=light_name)
        service_proxy_wrapper_mock.assert_called_once_with(GazeboServiceName.SET_LIGHT_PROPERTIES,
                                                           SetLightProperties)

    def test_initialize_with_custom_range(self, service_proxy_wrapper_mock):
        light_name = "test_light"
        color_range = {'r': {'min': 0.1, 'max': 0.4},
                       'g': {'min': 0.2, 'max': 0.5},
                       'b': {'min': 0.3, 'max': 0.6}}
        attenuation_range = {'constant': {'min': 0.4, 'max': 0.7},
                             'linear': {'min': 0.5, 'max': 0.8},
                             'quadratic': {'min': 0.6, 'max': 0.9}}

        light_randomizer = LightRandomizer(light_name=light_name,
                                           color_range=color_range,
                                           attenuation_range=attenuation_range)
        service_proxy_wrapper_mock.assert_called_once_with(GazeboServiceName.SET_LIGHT_PROPERTIES,
                                                           SetLightProperties)

        assert light_randomizer.light_name == light_name
        assert light_randomizer.color_range == color_range
        assert light_randomizer.attenuation_range == attenuation_range

    def test_randomize(self, service_proxy_wrapper_mock):
        light_name = "test_light"
        color_range = {'r': {'min': 0.1, 'max': 0.4},
                       'g': {'min': 0.2, 'max': 0.5},
                       'b': {'min': 0.3, 'max': 0.6}}
        attenuation_range = {'constant': {'min': 0.4, 'max': 0.7},
                             'linear': {'min': 0.5, 'max': 0.8},
                             'quadratic': {'min': 0.6, 'max': 0.9}}

        light_randomizer = LightRandomizer(light_name=light_name,
                                           color_range=color_range,
                                           attenuation_range=attenuation_range)
        with patch("deepsim.domain_randomizations.randomizers.light_randomizer.np.random.uniform") as uniform_random_mock:
            light_randomizer.randomize()
            uniform_random_mock.assert_has_calls([
                call(color_range['r']['min'], color_range['r']['max']),
                call(color_range['g']['min'], color_range['g']['max']),
                call(color_range['b']['min'], color_range['b']['max']),
                call(attenuation_range['constant']['min'], attenuation_range['constant']['max']),
                call(attenuation_range['linear']['min'], attenuation_range['linear']['max']),
                call(attenuation_range['quadratic']['min'], attenuation_range['quadratic']['max'])
            ])

