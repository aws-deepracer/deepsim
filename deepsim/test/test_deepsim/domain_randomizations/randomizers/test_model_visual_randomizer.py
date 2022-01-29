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
from deepsim.domain_randomizations.randomizers.model_visual_randomizer import ModelVisualRandomizer, ModelRandomizerType


@patch("deepsim.domain_randomizations.randomizers.model_visual_randomizer.ServiceProxyWrapper")
class ModelVisualRandomizerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, service_proxy_wrapper_mock):
        model_name = "test_model"
        model_randomizer_type = ModelRandomizerType.MODEL

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = ["body_name1"]

        get_visual_names_mock = MagicMock()
        get_visual_names_mock.return_value.visual_names = ["visual_name1"]

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock

        service_proxy_wrapper_mock.side_effect = service_proxy_creator
        model_visual_randomizer = ModelVisualRandomizer(model_name=model_name,
                                                        model_randomizer_type=model_randomizer_type)

        assert model_visual_randomizer.model_name == model_name
        assert model_visual_randomizer.model_randomizer_type == model_randomizer_type

    def test_initialize_custom_range(self, service_proxy_wrapper_mock):
        model_name = "test_model"
        model_randomizer_type = ModelRandomizerType.MODEL

        color_range = {'r': {'min': 0.1, 'max': 0.4},
                       'g': {'min': 0.2, 'max': 0.5},
                       'b': {'min': 0.3, 'max': 0.6}}

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = ["body_name1"]

        get_visual_names_mock = MagicMock()
        get_visual_names_mock.return_value.visual_names = ["visual_name1"]

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock

        service_proxy_wrapper_mock.side_effect = service_proxy_creator
        model_visual_randomizer = ModelVisualRandomizer(model_name=model_name,
                                                        model_randomizer_type=model_randomizer_type,
                                                        color_range=color_range)

        assert model_visual_randomizer.model_name == model_name
        assert model_visual_randomizer.model_randomizer_type == model_randomizer_type
        assert model_visual_randomizer.color_range == color_range

    def test_link_filter(self, service_proxy_wrapper_mock):
        model_name = "test_model"
        model_randomizer_type = ModelRandomizerType.MODEL

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = ["body_name1", "body_name2"]

        def get_visual_names(req):
            response_mock = MagicMock()
            response_mock.visual_names = []
            response_mock.link_names = []
            visual_names = ["visual_name1", "visual_name2"]
            for link_name in req.link_names:
                for visual_name in visual_names:
                    response_mock.link_names.append(link_name)
                    response_mock.visual_names.append(visual_name)
            return response_mock

        get_visual_names_mock = MagicMock()
        get_visual_names_mock.side_effect = get_visual_names

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock

        service_proxy_wrapper_mock.side_effect = service_proxy_creator
        model_visual_randomizer = ModelVisualRandomizer(model_name=model_name,
                                                        model_randomizer_type=model_randomizer_type,
                                                        link_name_filter=["test_model::body_name1"])
        assert len(model_visual_randomizer._link_visuals_map) == 1
        assert "test_model::body_name1" in model_visual_randomizer._link_visuals_map
        assert "test_model::body_name2" not in model_visual_randomizer._link_visuals_map

    def test_visual_filter(self, service_proxy_wrapper_mock):
        model_name = "test_model"
        model_randomizer_type = ModelRandomizerType.MODEL

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = ["body_name1", "body_name2"]

        def get_visual_names(req):
            response_mock = MagicMock()
            response_mock.visual_names = []
            response_mock.link_names = []
            visual_names = ["visual_name1", "visual_name2"]
            for link_name in req.link_names:
                for visual_name in visual_names:
                    response_mock.link_names.append(link_name)
                    response_mock.visual_names.append(visual_name)
            return response_mock

        get_visual_names_mock = MagicMock()
        get_visual_names_mock.side_effect = get_visual_names

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock

        service_proxy_wrapper_mock.side_effect = service_proxy_creator
        model_visual_randomizer = ModelVisualRandomizer(model_name=model_name,
                                                        model_randomizer_type=model_randomizer_type,
                                                        visual_name_filter=["visual_name1"])
        assert len(model_visual_randomizer._link_visuals_map) == 2
        for link_visual_names in model_visual_randomizer._link_visuals_map.values():
            assert "visual_name1" in link_visual_names
            assert "visual_name2" not in link_visual_names

    def test_model_randomizer_type_model(self, service_proxy_wrapper_mock):
        model_name = "test_model"
        model_randomizer_type = ModelRandomizerType.MODEL

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = ["body_name1", "body_name2"]

        def get_visual_names(req):
            response_mock = MagicMock()
            response_mock.visual_names = []
            response_mock.link_names = []
            visual_names = ["visual_name1", "visual_name2"]
            for link_name in req.link_names:
                for visual_name in visual_names:
                    response_mock.link_names.append(link_name)
                    response_mock.visual_names.append(visual_name)
            return response_mock

        get_visual_names_mock = MagicMock()
        get_visual_names_mock.side_effect = get_visual_names

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock

        service_proxy_wrapper_mock.side_effect = service_proxy_creator
        model_visual_randomizer = ModelVisualRandomizer(model_name=model_name,
                                                        model_randomizer_type=model_randomizer_type)
        with patch("deepsim.domain_randomizations.randomizers.model_visual_randomizer.SetVisualMaterialTracker") as tracker_mock:
            get_random_color_mock = MagicMock()
            model_visual_randomizer._get_random_color = get_random_color_mock
            model_visual_randomizer.randomize()
            assert get_random_color_mock.call_count == 1
            assert tracker_mock.get_instance.return_value.set_visual_material.call_count == 4

    def test_model_randomizer_type_link(self, service_proxy_wrapper_mock):
        model_name = "test_model"
        model_randomizer_type = ModelRandomizerType.LINK

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = ["body_name1", "body_name2"]

        def get_visual_names(req):
            response_mock = MagicMock()
            response_mock.visual_names = []
            response_mock.link_names = []
            visual_names = ["visual_name1", "visual_name2"]
            for link_name in req.link_names:
                for visual_name in visual_names:
                    response_mock.link_names.append(link_name)
                    response_mock.visual_names.append(visual_name)
            return response_mock

        get_visual_names_mock = MagicMock()
        get_visual_names_mock.side_effect = get_visual_names

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock

        service_proxy_wrapper_mock.side_effect = service_proxy_creator
        model_visual_randomizer = ModelVisualRandomizer(model_name=model_name,
                                                        model_randomizer_type=model_randomizer_type)
        with patch("deepsim.domain_randomizations.randomizers.model_visual_randomizer.SetVisualMaterialTracker") as tracker_mock:
            get_random_color_mock = MagicMock()
            model_visual_randomizer._get_random_color = get_random_color_mock
            model_visual_randomizer.randomize()
            assert get_random_color_mock.call_count == 3  # Last one is not used
            assert tracker_mock.get_instance.return_value.set_visual_material.call_count == 4

    def test_model_randomizer_type_visual(self, service_proxy_wrapper_mock):
        model_name = "test_model"
        model_randomizer_type = ModelRandomizerType.VISUAL

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = ["body_name1", "body_name2"]

        def get_visual_names(req):
            response_mock = MagicMock()
            response_mock.visual_names = []
            response_mock.link_names = []
            visual_names = ["visual_name1", "visual_name2"]
            for link_name in req.link_names:
                for visual_name in visual_names:
                    response_mock.link_names.append(link_name)
                    response_mock.visual_names.append(visual_name)
            return response_mock

        get_visual_names_mock = MagicMock()
        get_visual_names_mock.side_effect = get_visual_names

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock

        service_proxy_wrapper_mock.side_effect = service_proxy_creator
        model_visual_randomizer = ModelVisualRandomizer(model_name=model_name,
                                                        model_randomizer_type=model_randomizer_type)
        with patch("deepsim.domain_randomizations.randomizers.model_visual_randomizer.SetVisualMaterialTracker") as tracker_mock:
            get_random_color_mock = MagicMock()
            model_visual_randomizer._get_random_color = get_random_color_mock
            model_visual_randomizer.randomize()
            assert get_random_color_mock.call_count == 5  # Last one is not used
            assert tracker_mock.get_instance.return_value.set_visual_material.call_count == 4

    def test_model_randomizer_type_link_selection(self, service_proxy_wrapper_mock):
        model_name = "test_model"
        model_randomizer_type = ModelRandomizerType.LINK

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = ["body_name1", "body_name2"]

        def get_visual_names(req):
            response_mock = MagicMock()
            response_mock.visual_names = []
            response_mock.link_names = []
            visual_names = ["visual_name1", "visual_name2"]
            for link_name in req.link_names:
                for visual_name in visual_names:
                    response_mock.link_names.append(link_name)
                    response_mock.visual_names.append(visual_name)
            return response_mock

        get_visual_names_mock = MagicMock()
        get_visual_names_mock.side_effect = get_visual_names

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock

        service_proxy_wrapper_mock.side_effect = service_proxy_creator
        model_visual_randomizer = ModelVisualRandomizer(model_name=model_name,
                                                        model_randomizer_type=model_randomizer_type,
                                                        num_selection=1)
        with patch("deepsim.domain_randomizations.randomizers.model_visual_randomizer.SetVisualMaterialTracker") as tracker_mock:
            get_random_color_mock = MagicMock()
            model_visual_randomizer._get_random_color = get_random_color_mock
            model_visual_randomizer.randomize()
            assert get_random_color_mock.call_count == 2  # Last one is not used
            assert tracker_mock.get_instance.return_value.set_visual_material.call_count == 2

    def test_model_randomizer_type_visual_selection(self, service_proxy_wrapper_mock):
        model_name = "test_model"
        model_randomizer_type = ModelRandomizerType.VISUAL

        get_model_prop_mock = MagicMock()
        get_model_prop_mock.return_value.body_names = ["body_name1", "body_name2"]

        def get_visual_names(req):
            response_mock = MagicMock()
            response_mock.visual_names = []
            response_mock.link_names = []
            visual_names = ["visual_name1", "visual_name2"]
            for link_name in req.link_names:
                for visual_name in visual_names:
                    response_mock.link_names.append(link_name)
                    response_mock.visual_names.append(link_name + '_' + visual_name)
            return response_mock

        get_visual_names_mock = MagicMock()
        get_visual_names_mock.side_effect = get_visual_names

        def service_proxy_creator(service_name, service_class):
            if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                return get_model_prop_mock
            elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                return get_visual_names_mock

        service_proxy_wrapper_mock.side_effect = service_proxy_creator
        model_visual_randomizer = ModelVisualRandomizer(model_name=model_name,
                                                        model_randomizer_type=model_randomizer_type,
                                                        num_selection=3)
        with patch("deepsim.domain_randomizations.randomizers.model_visual_randomizer.SetVisualMaterialTracker") as tracker_mock:
            get_random_color_mock = MagicMock()
            model_visual_randomizer._get_random_color = get_random_color_mock
            model_visual_randomizer.randomize()
            assert get_random_color_mock.call_count == 4  # Last one is not used
            assert tracker_mock.get_instance.return_value.set_visual_material.call_count == 3
