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

from deepsim.visual_effects.effects.invisible_effect import InvisibleEffect
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.math.math import lerp

from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from deepsim_msgs.srv import (
    GetVisualNames, GetVisualNamesRequest,
    GetVisuals, GetVisualsRequest,
    SetVisualVisibles, SetVisualVisiblesRequest
)

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.visual_effects.effects.invisible_effect.ServiceProxyWrapper")
class InvisibleEffectTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, service_proxy_wrapper_mock):
        name = myself()
        duration = 1.0
        invisible_effect = InvisibleEffect(model_name=name,
                                           duration=duration)

        assert name == invisible_effect.model_name
        assert duration == invisible_effect.duration

    def test_lazy_init(self, service_proxy_wrapper_mock):
        name = myself()

        body_name = "body_name"
        link_name = "{}::{}".format(name, body_name)
        visual_name = "visual_name"

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = [body_name]
        get_visual_names_mock = MagicMock()
        get_visual_names_mock.return_value.link_names = [link_name]
        get_visual_names_mock.return_value.visual_names = [visual_name]
        get_visuals_mock = MagicMock()

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock
            elif service_name == GazeboServiceName.GET_VISUALS:
                return get_visuals_mock
            else:
                raise Exception("Unknown service name:{}".format(service_name))

        service_proxy_wrapper_mock.side_effect = service_proxy_creator

        invisible_effect = InvisibleEffect(model_name=name)
        invisible_effect._lazy_init()

        service_proxy_wrapper_mock.has_calls(
            call(GazeboServiceName.GET_MODEL_PROPERTIES, GetModelProperties),
            call(GazeboServiceName.GET_VISUAL_NAMES, GetVisualNames),
            call(GazeboServiceName.GET_VISUALS, GetVisuals)
        )

        get_model_prop_mock.assert_called_once_with(GetModelPropertiesRequest(model_name=name))
        get_visual_names_mock.assert_called_once_with(GetVisualNamesRequest(link_names=[link_name]))
        get_visuals_mock.assert_called_once_with(GetVisualsRequest(link_names=[link_name],
                                                                   visual_names=[visual_name]))

    def test_reset(self, service_proxy_wrapper_mock):
        name = myself()
        duration = 1.0
        invisible_effect = InvisibleEffect(model_name=name,
                                           duration=duration)
        invisible_effect._current_duration = 0.8

        invisible_effect.reset()
        assert invisible_effect._current_duration == 0.0

    def test_attach(self, service_proxy_wrapper_mock):
        name = myself()
        duration = 1.0
        invisible_effect = InvisibleEffect(model_name=name,
                                           duration=duration)
        invisible_effect._current_duration = 0.8

        with patch("deepsim.visual_effects.abs_effect.EffectManager") as effect_manager_mock:
            invisible_effect.attach()
            # Confirm reset is called
            assert invisible_effect._current_duration == 0.0
            assert invisible_effect._first_update_call
            effect_manager_mock.get_instance.return_value.add.assert_called_once_with(invisible_effect)

    def test_detach(self, service_proxy_wrapper_mock):
        name = myself()

        body_name = "body_name"
        link_name = "{}::{}".format(name, body_name)
        visual_name = "visual_name"

        body_name2 = "body_name2"
        link_name2 = "{}::{}".format(name, body_name)
        visual_name2 = "visual_name2"

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = [body_name, body_name2]
        get_visual_names_mock = MagicMock()
        get_visual_names_mock.return_value.link_names = [link_name, link_name2]
        get_visual_names_mock.return_value.visual_names = [visual_name, visual_name2]
        get_visuals_mock = MagicMock()
        get_visuals_mock.return_value.link_names = [link_name, link_name2]
        get_visuals_mock.return_value.visual_names = [visual_name, visual_name2]
        get_visuals_mock.return_value.visibles = [True, True]

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock
            elif service_name == GazeboServiceName.GET_VISUALS:
                return get_visuals_mock
            else:
                raise Exception("Unknown service name:{}".format(service_name))

        service_proxy_wrapper_mock.side_effect = service_proxy_creator

        invisible_effect = InvisibleEffect(model_name=name)

        with patch("deepsim.visual_effects.effects.invisible_effect.SetVisualVisibleTracker") as tracker_mock:
            invisible_effect.detach()

            tracker_mock.get_instance.return_value.set_visual_visible.has_calls(
                call(link_name, visual_name, True),
                call(link_name2, visual_name2, True)
            )

    def test_update(self, service_proxy_wrapper_mock):
        name = myself()

        body_name = "body_name"
        link_name = "{}::{}".format(name, body_name)
        visual_name = "visual_name"

        body_name2 = "body_name2"
        link_name2 = "{}::{}".format(name, body_name2)
        visual_name2 = "visual_name2"

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = [body_name, body_name2]
        get_visual_names_mock = MagicMock()
        get_visual_names_mock.return_value.link_names = [link_name, link_name2]
        get_visual_names_mock.return_value.visual_names = [visual_name, visual_name2]
        get_visuals_mock = MagicMock()
        get_visuals_mock.return_value.link_names = [link_name, link_name2]
        get_visuals_mock.return_value.visual_names = [visual_name, visual_name2]
        get_visuals_mock.return_value.visibles = [True, True]

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock
            elif service_name == GazeboServiceName.GET_VISUALS:
                return get_visuals_mock
            else:
                raise Exception("Unknown service name:{}".format(service_name))

        service_proxy_wrapper_mock.side_effect = service_proxy_creator

        duration = 3.0

        blink_effect = InvisibleEffect(model_name=name,
                                       duration=duration)
        blink_effect.attach()
        delta_time = 0.15
        sim_time_mock = MagicMock()

        with patch("deepsim.visual_effects.effects.invisible_effect.SetVisualVisibleTracker") as tracker_mock:
            blink_effect.update(delta_time=delta_time,
                                sim_time=sim_time_mock)

            # Test Lazy Init called during update
            service_proxy_wrapper_mock.has_calls(
                call(GazeboServiceName.GET_MODEL_PROPERTIES, GetModelProperties),
                call(GazeboServiceName.GET_VISUAL_NAMES, GetVisualNames),
                call(GazeboServiceName.GET_VISUALS, GetVisuals)
            )

            get_model_prop_mock.assert_called_once_with(GetModelPropertiesRequest(model_name=name))
            get_visual_names_mock.assert_called_once_with(GetVisualNamesRequest(link_names=[link_name, link_name2]))
            get_visuals_mock.assert_called_once_with(GetVisualsRequest(link_names=[link_name, link_name2],
                                                                       visual_names=[visual_name, visual_name2]))

            # Test set visual transparency called
            assert tracker_mock.get_instance.return_value.set_visual_visible.call_count == 2
            tracker_mock.get_instance.return_value.set_visual_visible.has_calls(
                call(link_name, visual_name, False),
                call(link_name2, visual_name2, False))
            assert blink_effect.current_duration == delta_time

    def test_update_duration_passed(self, service_proxy_wrapper_mock):
        name = myself()

        body_name = "body_name"
        link_name = "{}::{}".format(name, body_name)
        visual_name = "visual_name"
        transparency = 0.0

        body_name2 = "body_name2"
        link_name2 = "{}::{}".format(name, body_name2)
        visual_name2 = "visual_name2"
        transparency2 = 0.5

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = [body_name, body_name2]
        get_visual_names_mock = MagicMock()
        get_visual_names_mock.return_value.link_names = [link_name, link_name2]
        get_visual_names_mock.return_value.visual_names = [visual_name, visual_name2]
        get_visuals_mock = MagicMock()
        get_visuals_mock.return_value.link_names = [link_name, link_name2]
        get_visuals_mock.return_value.visual_names = [visual_name, visual_name2]
        get_visuals_mock.return_value.transparencies = [transparency, transparency2]

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock
            elif service_name == GazeboServiceName.GET_VISUALS:
                return get_visuals_mock
            else:
                raise Exception("Unknown service name:{}".format(service_name))

        service_proxy_wrapper_mock.side_effect = service_proxy_creator

        invisible_effect = InvisibleEffect(model_name=name)
        invisible_effect.attach()
        # Set current duration to the end
        invisible_effect._current_interval = 0.9
        invisible_effect._current_duration = 2.0
        # Set delta time to pass the interval
        delta_time = 0.15
        sim_time_mock = MagicMock()

        with patch("deepsim.visual_effects.effects.invisible_effect.SetVisualVisibleTracker") as tracker_mock:
            invisible_effect.update(delta_time=delta_time,
                                    sim_time=sim_time_mock)
            # Confirm next update detached the effect.
            tracker_mock.get_instance.return_value.set_visual_visible.has_calls(
                call(link_name, visual_name, True),
                call(link_name2, visual_name2, True)
            )

