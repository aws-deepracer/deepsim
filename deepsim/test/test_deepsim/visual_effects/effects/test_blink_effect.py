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

from deepsim.visual_effects.effects.blink_effect import BlinkEffect
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.math.math import lerp

from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from deepsim_msgs.srv import (
    GetVisualNames, GetVisualNamesRequest,
    GetVisuals, GetVisualsRequest
)

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.visual_effects.effects.blink_effect.ServiceProxyWrapper")
class BlinkEffectTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, service_proxy_wrapper_mock):
        name = myself()
        min_alpha = 0.4
        max_alpha = 0.9
        interval = 0.5
        duration = 1.0
        blink_effect = BlinkEffect(model_name=name,
                                   min_alpha=min_alpha,
                                   max_alpha=max_alpha,
                                   interval=interval,
                                   duration=duration)

        assert name == blink_effect.model_name
        assert min_alpha == blink_effect.min_alpha
        assert max_alpha == blink_effect.max_alpha
        assert interval == blink_effect.interval
        assert duration == blink_effect.duration

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

        blink_effect = BlinkEffect(model_name=name)
        blink_effect._lazy_init()

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
        min_alpha = 0.4
        max_alpha = 0.9
        interval = 0.5
        duration = 1.0
        blink_effect = BlinkEffect(model_name=name,
                                   min_alpha=min_alpha,
                                   max_alpha=max_alpha,
                                   interval=interval,
                                   duration=duration)
        blink_effect._source_alpha = 0.5
        blink_effect._target_alpha = 0.6
        blink_effect._current_interval = 0.7
        blink_effect._current_duration = 0.8

        blink_effect.reset()
        assert blink_effect._source_alpha == blink_effect.min_alpha
        assert blink_effect._target_alpha == blink_effect.max_alpha
        assert blink_effect._current_interval == 0.0
        assert blink_effect._current_duration == 0.0

    def test_attach(self, service_proxy_wrapper_mock):
        name = myself()
        min_alpha = 0.4
        max_alpha = 0.9
        interval = 0.5
        duration = 1.0
        blink_effect = BlinkEffect(model_name=name,
                                   min_alpha=min_alpha,
                                   max_alpha=max_alpha,
                                   interval=interval,
                                   duration=duration)
        blink_effect._source_alpha = 0.5
        blink_effect._target_alpha = 0.6
        blink_effect._current_interval = 0.7
        blink_effect._current_duration = 0.8

        with patch("deepsim.visual_effects.abs_effect.EffectManager") as effect_manager_mock:
            blink_effect.attach()
            assert blink_effect._source_alpha == blink_effect.min_alpha
            assert blink_effect._target_alpha == blink_effect.max_alpha
            assert blink_effect._current_interval == 0.0
            assert blink_effect._current_duration == 0.0

            effect_manager_mock.get_instance.return_value.add.assert_called_once_with(blink_effect)

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

        blink_effect = BlinkEffect(model_name=name)

        with patch("deepsim.visual_effects.effects.blink_effect.SetVisualTransparencyTracker") as tracker_mock:
            blink_effect.detach()
            tracker_mock.get_instance.return_value.set_visual_transparency.has_calls(
                call(link_name, visual_name, 0.0),
                call(link_name2, visual_name2, 0.0)
            )

    def test_update(self, service_proxy_wrapper_mock):
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

        min_alpha = 0.3
        max_alpha = 0.8
        interval = 2.0
        duration = 3.0

        blink_effect = BlinkEffect(model_name=name,
                                   min_alpha=min_alpha,
                                   max_alpha=max_alpha,
                                   interval=interval,
                                   duration=duration)
        blink_effect.attach()
        delta_time = 0.15
        sim_time_mock = MagicMock()

        cur_alpha = lerp(min_alpha, max_alpha, 0.0 / interval)
        transparency = 1.0 - cur_alpha
        transparencies = [transparency]

        with patch("deepsim.visual_effects.effects.blink_effect.SetVisualTransparencyTracker") as tracker_mock:
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
            assert tracker_mock.get_instance.return_value.set_visual_transparency.call_count == 2
            tracker_mock.get_instance.return_value.set_visual_transparency.has_calls(
                call(link_name, visual_name, transparency),
                call(link_name2, visual_name2, transparency2))
            assert blink_effect.current_interval == delta_time
            assert blink_effect.current_duration == delta_time

    def test_update_interval_passed(self, service_proxy_wrapper_mock):
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

        min_alpha = 0.3
        max_alpha = 0.8
        interval = 2.0
        duration = 3.0

        blink_effect = BlinkEffect(model_name=name,
                                   min_alpha=min_alpha,
                                   max_alpha=max_alpha,
                                   interval=interval,
                                   duration=duration)
        blink_effect.attach()
        # Set current interval to almost at the end
        blink_effect._current_interval = 1.9
        blink_effect._current_duration = 1.9
        # Set delta time to pass the interval
        delta_time = 0.15
        sim_time_mock = MagicMock()

        cur_alpha = lerp(min_alpha, max_alpha, blink_effect._current_interval / interval)
        transparency = 1.0 - cur_alpha
        transparencies = [transparency]

        with patch("deepsim.visual_effects.effects.blink_effect.SetVisualTransparencyTracker") as tracker_mock:
            source_alpha = blink_effect._source_alpha
            target_alpha = blink_effect._target_alpha
            current_duration = blink_effect.current_duration
            blink_effect.update(delta_time=delta_time,
                                sim_time=sim_time_mock)

            # Test set visual transparency called
            assert tracker_mock.get_instance.return_value.set_visual_transparency.call_count == 2
            tracker_mock.get_instance.return_value.set_visual_transparency.has_calls(
                call(link_name, visual_name, transparency),
                call(link_name2, visual_name2, transparency2))
            assert blink_effect._source_alpha == target_alpha
            assert blink_effect._target_alpha == source_alpha
            assert blink_effect.current_interval == 0.0
            assert blink_effect._current_duration == current_duration + delta_time

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

        blink_effect = BlinkEffect(model_name=name)
        blink_effect.attach()
        # Set current duration to the end
        blink_effect._current_interval = 0.9
        blink_effect._current_duration = 2.0
        # Set delta time to pass the interval
        delta_time = 0.15
        sim_time_mock = MagicMock()

        with patch("deepsim.visual_effects.effects.blink_effect.SetVisualTransparencyTracker") as tracker_mock:
            blink_effect.update(delta_time=delta_time,
                                sim_time=sim_time_mock)
            # Confirm next update detached the effect.
            tracker_mock.get_instance.return_value.set_visual_transparency.has_calls(
                call(link_name, visual_name, 0.0),
                call(link_name2, visual_name2, 0.0)
            )

