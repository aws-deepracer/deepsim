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

from deepsim.sim_trackers.trackers.set_visual_material_tracker import SetVisualMaterialTracker
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.math.color import Color
from deepsim.math.material import Material

from deepsim_msgs.srv import (
    SetVisualMaterials, SetVisualMaterialsRequest, SetVisualMaterialsResponse,
    SetVisualMaterial, SetVisualMaterialResponse
)

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.sim_trackers.trackers.set_visual_material_tracker.GetVisualTracker")
@patch("deepsim.sim_trackers.trackers.set_visual_material_tracker.ServiceProxyWrapper")
class SetVisualMaterialStateTrackerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        _ = SetVisualMaterialTracker(is_singleton=False)
        service_proxy_wrapper_mock.assert_called_once_with(GazeboServiceName.SET_VISUAL_MATERIALS,
                                                           SetVisualMaterials)

    def test_set_visual_material(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualMaterialTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        tracker.set_visual_material(link_name=link_name,
                                    visual_name=visual_name,
                                    material=material)
        key = (link_name, visual_name)
        assert tracker._visual_name_map[key] == visual_name
        assert tracker._link_name_map[key] == link_name
        assert tracker._ambient_map[key] == ambient.to_ros()
        assert tracker._diffuse_map[key] == diffuse.to_ros()
        assert tracker._specular_map[key] == specular.to_ros()
        assert tracker._emissive_map[key] == emissive.to_ros()

    def test_set_visual_material_blocking(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualMaterialTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        tracker.set_visual_material(link_name=link_name,
                                    visual_name=visual_name,
                                    material=material)

        res = SetVisualMaterialsResponse()
        res.success = True
        res.status.append(True)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetVisualMaterialResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        msg = tracker.set_visual_material(link_name=link_name,
                                          visual_name=visual_name,
                                          material=material,
                                          blocking=True)

        req = SetVisualMaterialsRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.ambients = [material.ambient.to_ros()]
        req.diffuses = [material.diffuse.to_ros()]
        req.speculars = [material.specular.to_ros()]
        req.emissives = [material.emissive.to_ros()]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        get_visual_tracker_mock_obj = get_visual_tracker_mock.get_instance.return_value
        get_visual_tracker_mock_obj.set_material.assert_called_once_with(link_name=link_name,
                                                                         visual_name=visual_name,
                                                                         material=material)

    def test_set_visual_material_blocking_override(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualMaterialTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        res = SetVisualMaterialsResponse()
        res.success = True
        res.status.append(True)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetVisualMaterialResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        _ = tracker.set_visual_material(link_name=link_name,
                                        visual_name=visual_name,
                                        material=material)

        # Confirm new model_state is persisted to map
        key = (link_name, visual_name)
        assert tracker._visual_name_map[key] == visual_name
        assert tracker._link_name_map[key] == link_name
        assert tracker._ambient_map[key] == ambient.to_ros()
        assert tracker._diffuse_map[key] == diffuse.to_ros()
        assert tracker._specular_map[key] == specular.to_ros()
        assert tracker._emissive_map[key] == emissive.to_ros()

        msg = tracker.set_visual_material(link_name=link_name,
                                          visual_name=visual_name,
                                          material=material,
                                          blocking=True)

        # Confirm the service is called with new model state.
        req = SetVisualMaterialsRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.ambients = [material.ambient.to_ros()]
        req.diffuses = [material.diffuse.to_ros()]
        req.speculars = [material.specular.to_ros()]
        req.emissives = [material.emissive.to_ros()]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        # Confirm the new model state is removed from the map.
        key = (link_name, visual_name)
        assert key not in tracker._visual_name_map
        assert key not in tracker._link_name_map
        assert key not in tracker._ambient_map
        assert key not in tracker._diffuse_map
        assert key not in tracker._specular_map
        assert key not in tracker._emissive_map
        get_visual_tracker_mock_obj = get_visual_tracker_mock.get_instance.return_value
        get_visual_tracker_mock_obj.set_material.assert_called_once_with(link_name=link_name,
                                                                         visual_name=visual_name,
                                                                         material=material)

    def test_set_visual_material_blocking_failed(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualMaterialTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        tracker.set_visual_material(link_name=link_name,
                                    visual_name=visual_name,
                                    material=material)

        res = SetVisualMaterialsResponse()
        res.success = False
        res.status.append(False)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetVisualMaterialResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        msg = tracker.set_visual_material(link_name=link_name,
                                          visual_name=visual_name,
                                          material=material,
                                          blocking=True)

        req = SetVisualMaterialsRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.ambients = [material.ambient.to_ros()]
        req.diffuses = [material.diffuse.to_ros()]
        req.speculars = [material.specular.to_ros()]
        req.emissives = [material.emissive.to_ros()]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        get_visual_tracker_mock.get_instance.return_value.set_material.assert_not_called()

    def test_set_visual_materials(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualMaterialTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        tracker.set_visual_materials(link_names=[link_name],
                                     visual_names=[visual_name],
                                     materials=[material])
        key = (link_name, visual_name)
        assert tracker._visual_name_map[key] == visual_name
        assert tracker._link_name_map[key] == link_name
        assert tracker._ambient_map[key] == ambient.to_ros()
        assert tracker._diffuse_map[key] == diffuse.to_ros()
        assert tracker._specular_map[key] == specular.to_ros()
        assert tracker._emissive_map[key] == emissive.to_ros()

    def test_set_visual_materials_blocking(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualMaterialTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        tracker.set_visual_materials(link_names=[link_name],
                                     visual_names=[visual_name],
                                     materials=[material])

        expected_msg = SetVisualMaterialsResponse()
        expected_msg.success = True
        expected_msg.status.append(True)
        expected_msg.messages.append("")

        service_proxy_wrapper_mock.return_value.return_value = expected_msg
        msg = tracker.set_visual_materials(link_names=[link_name],
                                           visual_names=[visual_name],
                                           materials=[material],
                                           blocking=True)

        req = SetVisualMaterialsRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.ambients = [material.ambient.to_ros()]
        req.diffuses = [material.diffuse.to_ros()]
        req.speculars = [material.specular.to_ros()]
        req.emissives = [material.emissive.to_ros()]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        get_visual_tracker_mock_obj = get_visual_tracker_mock.get_instance.return_value
        get_visual_tracker_mock_obj.set_material.assert_called_once_with(link_name=link_name,
                                                                         visual_name=visual_name,
                                                                         material=material)

    def test_set_visual_materials_blocking_failed(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualMaterialTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        tracker.set_visual_materials(link_names=[link_name],
                                     visual_names=[visual_name],
                                     materials=[material])

        expected_msg = SetVisualMaterialsResponse()
        expected_msg.success = False
        expected_msg.status.append(False)
        expected_msg.messages.append("")

        service_proxy_wrapper_mock.return_value.return_value = expected_msg
        msg = tracker.set_visual_materials(link_names=[link_name],
                                           visual_names=[visual_name],
                                           materials=[material],
                                           blocking=True)

        req = SetVisualMaterialsRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.ambients = [material.ambient.to_ros()]
        req.diffuses = [material.diffuse.to_ros()]
        req.speculars = [material.specular.to_ros()]
        req.emissives = [material.emissive.to_ros()]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        get_visual_tracker_mock.get_instance.return_value.set_material.assert_not_called()

    def test_on_update_tracker(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualMaterialTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        res = SetVisualMaterialsResponse()
        res.success = True
        res.status.append(True)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetVisualMaterialResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        _ = tracker.set_visual_material(link_name=link_name,
                                        visual_name=visual_name,
                                        material=material)

        # Confirm new model_state is persisted to map
        key = (link_name, visual_name)
        assert tracker._visual_name_map[key] == visual_name
        assert tracker._link_name_map[key] == link_name
        assert tracker._ambient_map[key] == ambient.to_ros()
        assert tracker._diffuse_map[key] == diffuse.to_ros()
        assert tracker._specular_map[key] == specular.to_ros()
        assert tracker._emissive_map[key] == emissive.to_ros()

        tracker.on_update_tracker(MagicMock(), MagicMock())
        # Confirm the service is called with new model state.
        req = SetVisualMaterialsRequest()

        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.ambients = [ambient.to_ros()]
        req.diffuses = [diffuse.to_ros()]
        req.speculars = [specular.to_ros()]
        req.emissives = [emissive.to_ros()]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)

        key = (link_name, visual_name)
        assert key not in tracker._visual_name_map
        assert key not in tracker._link_name_map
        assert key not in tracker._ambient_map
        assert key not in tracker._diffuse_map
        assert key not in tracker._specular_map
        assert key not in tracker._emissive_map
