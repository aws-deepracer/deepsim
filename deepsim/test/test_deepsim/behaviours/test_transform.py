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

import math

from deepsim.behaviours.transform import Transform
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.math.color import Color
from deepsim.math.pose import Pose
from deepsim.math.twist import Twist
from deepsim.math.vector3 import Vector3
from deepsim.math.quaternion import Quaternion
from deepsim.math.euler import Euler
from deepsim.math.material import Material
from deepsim.math.model_state import ModelState
from deepsim.exception import DeepSimException

from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from deepsim_msgs.srv import (
    GetVisualNames, GetVisualNamesRequest, GetVisualNamesResponse,
    GetVisuals, GetVisualsRequest, GetVisualsResponse
)

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.behaviours.transform.ROSUtil")
@patch("deepsim.behaviours.transform.ServiceProxyWrapper")
class TransformTest(TestCase):
    def setUp(self) -> None:
        def service_proxy_mock_creator(link_names, visual_names):
            get_model_property_mock = MagicMock()
            get_model_property_mock.return_value.body_names = ["test_body_name"]

            get_visual_names_mock = MagicMock()
            response = GetVisualNamesResponse()
            response.link_names = link_names
            response.visual_names = visual_names
            get_visual_names_mock.return_value = response

            def service_proxy_creator(service_name, service_class):
                if service_name == GazeboServiceName.GET_MODEL_PROPERTIES:
                    return get_model_property_mock
                elif service_name == GazeboServiceName.GET_VISUAL_NAMES:
                    return get_visual_names_mock

            return service_proxy_creator

        self.service_proxy_mock_creator = service_proxy_mock_creator
        self.link_name_format = "{}::test_body_name"
        self.visual_name = "test_visual_name"

    def test_initialize(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        key = (link_names[0], visual_names[0])

        behaviour_mock = MagicMock()
        name = myself()
        transform = Transform(behaviour_mock, name)
        service_proxy_wrapper_mock.assert_has_calls([
            call(GazeboServiceName.GET_MODEL_PROPERTIES,
                 GetModelProperties),
            call(GazeboServiceName.GET_VISUAL_NAMES,
                 GetVisualNames),
        ])
        assert transform.behaviour == behaviour_mock
        assert transform.name == name
        assert transform._local_state == ModelState(model_name=name)
        assert transform._local_materials == {key: Material()}
        assert transform._local_visibles == {key: False}

        assert transform._local_transparencies == {key: 0.0}
        assert name in Transform.transforms

    def test_vectors(self, service_proxy_wrapper_mock, ros_util_mock):
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.GetModelStateTracker") as get_model_state_tracker_mock:
            euler = Euler(0.0, math.pi / 2.0, math.pi / 2.0)
            ret_model_state = ModelState(pose=Pose(position=Vector3(0.0, 0.0, 0.0),
                                                   orientation=euler.to_quaternion()))
            get_model_state_tracker_mock.get_instance.return_value.get_model_state.return_value = ret_model_state
            transform = Transform(behaviour_mock, name)
            assert transform.forward == Vector3.forward().rotate(euler.to_quaternion())
            assert transform.forward == Vector3(0.0, 0.0, -1.0)
            assert transform.back == Vector3.back().rotate(euler.to_quaternion())
            assert transform.back == Vector3(0.0, 0.0, 1.0)
            assert transform.left == Vector3.left().rotate(euler.to_quaternion())
            assert transform.left == Vector3(-1.0, 0.0, 0.0)
            assert transform.right == Vector3.right().rotate(euler.to_quaternion())
            assert transform.right == Vector3(1.0, 0.0, 0.0)
            assert transform.up == Vector3.up().rotate(euler.to_quaternion())
            assert transform.up == Vector3(0.0, 1.0, 0.0)
            assert transform.down == Vector3.down().rotate(euler.to_quaternion())
            assert transform.down == Vector3(0.0, -1.0, 0.0)

    def test_get_state(self, service_proxy_wrapper_mock, ros_util_mock):
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.GetModelStateTracker") as get_model_state_tracker_mock:
            transform = Transform(behaviour_mock, name)

            _ = transform.get_state()
            get_model_state_tracker_mock_obj = get_model_state_tracker_mock.get_instance.return_value
            get_model_state_tracker_mock_obj.get_model_state.assert_called_once_with(name,
                                                                                     reference_frame=None,
                                                                                     blocking=False)

    def test_get_state_blocking(self, service_proxy_wrapper_mock, ros_util_mock):
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.GetModelStateTracker") as get_model_state_tracker_mock:
            transform = Transform(behaviour_mock, name)

            _ = transform.get_state(blocking=True)
            get_model_state_tracker_mock_obj = get_model_state_tracker_mock.get_instance.return_value
            get_model_state_tracker_mock_obj.get_model_state.assert_called_once_with(name,
                                                                                     reference_frame=None,
                                                                                     blocking=True)

    def test_get_state_no_model(self, service_proxy_wrapper_mock, ros_util_mock):
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.GetModelStateTracker") as get_model_state_tracker_mock:
            transform = Transform(behaviour_mock, name)
            get_model_state_tracker_mock.get_instance.return_value.get_model_state.side_effect = DeepSimException()
            model_state = transform.get_state()
            assert model_state == ModelState(model_name=name)

    def test_get_state_no_model_after_set(self, service_proxy_wrapper_mock, ros_util_mock):
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.GetModelStateTracker") as get_model_state_tracker_mock, \
                patch("deepsim.behaviours.transform.SetModelStateTracker") as set_model_state_tracker_mock:
            transform = Transform(behaviour_mock, name)

            new_state = ModelState(pose=Pose(position=Vector3(1.0, 2.0, 3.0),
                                             orientation=Quaternion(2.0, 3.0, 4.0, 5.0)),
                                   twist=Twist(linear=Vector3(3.0, 4.0, 5.0),
                                               angular=Vector3(4.0, 5.0, 6.0)))
            transform.state = new_state
            expected_state = new_state.copy()
            expected_state.model_name = transform.name
            get_model_state_tracker_mock.get_instance.return_value.get_model_state.side_effect = DeepSimException()
            model_state = transform.get_state()
            assert model_state == expected_state

    def test_set_state(self, service_proxy_wrapper_mock, ros_util_mock):
        mode_state_mock = MagicMock()
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.SetModelStateTracker") as set_model_state_tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.set_state(mode_state_mock)

            expected_model_state = mode_state_mock.copy.return_value

            set_model_state_tracker_mock_obj = set_model_state_tracker_mock.get_instance.return_value
            set_model_state_tracker_mock_obj.set_model_state.assert_called_once_with(expected_model_state,
                                                                                     blocking=False)
            mode_state_mock.copy.assert_called_once()
            assert expected_model_state.model_name == name
            assert transform._local_state == expected_model_state

    def test_set_stat_blocking(self, service_proxy_wrapper_mock, ros_util_mock):
        mode_state_mock = MagicMock()
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.SetModelStateTracker") as set_model_state_tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.set_state(mode_state_mock, blocking=True)

            expected_model_state = mode_state_mock.copy.return_value

            set_model_state_tracker_mock_obj = set_model_state_tracker_mock.get_instance.return_value
            set_model_state_tracker_mock_obj.set_model_state.assert_called_once_with(expected_model_state,
                                                                                     blocking=True)
            mode_state_mock.copy.assert_called_once()
            assert expected_model_state.model_name == name
            assert transform._local_state == expected_model_state

    def test_state(self, service_proxy_wrapper_mock, ros_util_mock):
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.GetModelStateTracker") as get_model_state_tracker_mock:
            transform = Transform(behaviour_mock, name)

            _ = transform.state
            get_model_state_tracker_mock_obj = get_model_state_tracker_mock.get_instance.return_value
            get_model_state_tracker_mock_obj.get_model_state.assert_called_once_with(name,
                                                                                     reference_frame=None,
                                                                                     blocking=False)

    def test_state_setter(self, service_proxy_wrapper_mock, ros_util_mock):
        mode_state_mock = MagicMock()
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.SetModelStateTracker") as set_model_state_tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.state = mode_state_mock

            expected_model_state = mode_state_mock.copy.return_value

            set_model_state_tracker_mock_obj = set_model_state_tracker_mock.get_instance.return_value
            set_model_state_tracker_mock_obj.set_model_state.assert_called_once_with(expected_model_state,
                                                                                     blocking=False)
            mode_state_mock.copy.assert_called_once()
            assert expected_model_state.model_name == name
            assert transform._local_state == expected_model_state

    def test_get_material(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        ambient = Color(0.1, 0.1, 0.1, 0.1)
        diffuse = Color(0.2, 0.2, 0.2, 0.2)
        specular = Color(0.3, 0.3, 0.3, 0.3)
        emissive = Color(0.4, 0.4, 0.4, 0.4)

        key = (link_names[0], visual_names[0])
        expected_material = {key: Material(ambient=ambient,
                                           diffuse=diffuse,
                                           specular=specular,
                                           emissive=emissive)}

        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.material = Material(ambient=ambient,
                                            diffuse=diffuse,
                                            specular=specular,
                                            emissive=emissive)
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)

            returned_material = transform.get_material()
            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=False)

            assert returned_material == expected_material

    def test_get_material_blocking(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        ambient = Color(0.1, 0.1, 0.1, 0.1)
        diffuse = Color(0.2, 0.2, 0.2, 0.2)
        specular = Color(0.3, 0.3, 0.3, 0.3)
        emissive = Color(0.4, 0.4, 0.4, 0.4)
        key = (link_names[0], visual_names[0])
        expected_material = {key: Material(ambient=ambient,
                                           diffuse=diffuse,
                                           specular=specular,
                                           emissive=emissive)}

        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.material = Material(ambient=ambient,
                                            diffuse=diffuse,
                                            specular=specular,
                                            emissive=emissive)
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)

            returned_material = transform.get_material(blocking=True)
            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=True)

            assert returned_material == expected_material

    def test_get_material_no_model(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        expected_material = {}

        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False

            transform = Transform(behaviour_mock, name)
            returned_material = transform.get_material()
            tracker_mock.get_instance.return_value.get_visuals.assert_not_called()
            assert returned_material == expected_material

    def test_get_material_no_model_after_set(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False
            transform = Transform(behaviour_mock, name)

            ambient = Color(0.1, 0.1, 0.1, 0.1)
            diffuse = Color(0.2, 0.2, 0.2, 0.2)
            specular = Color(0.3, 0.3, 0.3, 0.3)
            emissive = Color(0.4, 0.4, 0.4, 0.4)

            expected_material = Material(ambient=ambient,
                                         diffuse=diffuse,
                                         specular=specular,
                                         emissive=emissive)

            transform.material = expected_material

            returned_material = transform.get_material()
            tracker_mock.get_instance.return_value.get_visuals.assert_not_called()
            assert returned_material == {}

    def test_set_material(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        ambient = Color(0.1, 0.1, 0.1, 0.1)
        diffuse = Color(0.2, 0.2, 0.2, 0.2)
        specular = Color(0.3, 0.3, 0.3, 0.3)
        emissive = Color(0.4, 0.4, 0.4, 0.4)

        expected_material = Material(ambient=ambient,
                                     diffuse=diffuse,
                                     specular=specular,
                                     emissive=emissive)
        key = (link_names[0], visual_names[0])
        expected_return = {key: expected_material}
        with patch("deepsim.behaviours.transform.SetVisualMaterialTracker") as tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.set_material(expected_material)
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_materials.assert_called_once_with(
                link_names=[self.link_name_format.format(myself())],
                visual_names=[self.visual_name],
                materials=[expected_material],
                blocking=False)
            assert transform._local_materials == expected_return

    def test_set_material_blocking(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        ambient = Color(0.1, 0.1, 0.1, 0.1)
        diffuse = Color(0.2, 0.2, 0.2, 0.2)
        specular = Color(0.3, 0.3, 0.3, 0.3)
        emissive = Color(0.4, 0.4, 0.4, 0.4)

        expected_material = Material(ambient=ambient,
                                     diffuse=diffuse,
                                     specular=specular,
                                     emissive=emissive)
        key = (link_names[0], visual_names[0])
        expected_return = {key: expected_material}

        with patch("deepsim.behaviours.transform.SetVisualMaterialTracker") as tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.set_material(expected_material, blocking=True)
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_materials.assert_called_once_with(
                link_names=[self.link_name_format.format(myself())],
                visual_names=[self.visual_name],
                materials=[expected_material],
                blocking=True)
            assert transform._local_materials == expected_return

    def test_material_setter_no_model(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        ambient = Color(0.1, 0.1, 0.1, 0.1)
        diffuse = Color(0.2, 0.2, 0.2, 0.2)
        specular = Color(0.3, 0.3, 0.3, 0.3)
        emissive = Color(0.4, 0.4, 0.4, 0.4)

        expected_material = Material(ambient=ambient,
                                     diffuse=diffuse,
                                     specular=specular,
                                     emissive=emissive)
        key = (link_names[0], visual_names[0])
        expected_return = {key: expected_material}

        with patch("deepsim.behaviours.transform.SetVisualMaterialTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False
            transform = Transform(behaviour_mock, name)
            transform.material = expected_material
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_materials.assert_not_called()
            assert transform._local_materials == {}

    def test_material(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        ambient = Color(0.1, 0.1, 0.1, 0.1)
        diffuse = Color(0.2, 0.2, 0.2, 0.2)
        specular = Color(0.3, 0.3, 0.3, 0.3)
        emissive = Color(0.4, 0.4, 0.4, 0.4)

        key = (link_names[0], visual_names[0])
        expected_material = {key: Material(ambient=ambient,
                                           diffuse=diffuse,
                                           specular=specular,
                                           emissive=emissive)}

        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.material = Material(ambient=ambient,
                                            diffuse=diffuse,
                                            specular=specular,
                                            emissive=emissive)
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)

            returned_material = transform.material

            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=False)
            assert returned_material == expected_material

    def test_material_setter(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        ambient = Color(0.1, 0.1, 0.1, 0.1)
        diffuse = Color(0.2, 0.2, 0.2, 0.2)
        specular = Color(0.3, 0.3, 0.3, 0.3)
        emissive = Color(0.4, 0.4, 0.4, 0.4)

        expected_material = Material(ambient=ambient,
                                     diffuse=diffuse,
                                     specular=specular,
                                     emissive=emissive)
        key = (link_names[0], visual_names[0])
        expected_return = {key: expected_material}
        with patch("deepsim.behaviours.transform.SetVisualMaterialTracker") as tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.material = expected_material
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_materials.assert_called_once_with(
                link_names=[self.link_name_format.format(myself())],
                visual_names=[self.visual_name],
                materials=[expected_material],
                blocking=False)
            assert transform._local_materials == expected_return

    def test_get_transparency(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])

        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.transparency = 0.5
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            assert transform.get_transparency() == {key: 0.5}
            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=False)

    def test_get_transparency_blocking(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.transparency = 0.5
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            assert transform.get_transparency(blocking=True) == {key: 0.5}
            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=True)

    def test_get_transparency_no_model(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False

            visual_mock = MagicMock()
            visual_mock.transparency = 0.5
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            assert transform.get_transparency() == {}
            tracker_mock.get_instance.return_value.get_visuals.assert_not_called()

    def test_get_transparency_no_model_after_set(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False

            visual_mock = MagicMock()
            visual_mock.transparency = 0.5
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            transform.transparency = 0.6
            assert transform.get_transparency() == {}
            tracker_mock.get_instance.return_value.get_visuals.assert_not_called()

    def test_set_transparency(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        key = (link_names[0], visual_names[0])
        expected_transparency = 0.5
        expected_return = {key: expected_transparency}
        with patch("deepsim.behaviours.transform.SetVisualTransparencyTracker") as tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.set_transparency(expected_transparency)
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_transparencies.assert_called_once_with(
                link_names=[self.link_name_format.format(myself())],
                visual_names=[self.visual_name],
                transparencies=[expected_transparency],
                blocking=False)
            assert transform._local_transparencies == expected_return

    def test_set_transparency_blocking(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        key = (link_names[0], visual_names[0])
        expected_transparency = 0.5
        expected_return = {key: expected_transparency}
        with patch("deepsim.behaviours.transform.SetVisualTransparencyTracker") as tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.set_transparency(expected_transparency,
                                       blocking=True)
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_transparencies.assert_called_once_with(
                link_names=[self.link_name_format.format(myself())],
                visual_names=[self.visual_name],
                transparencies=[expected_transparency],
                blocking=True)
            assert transform._local_transparencies == expected_return

    def test_set_transparency_no_model(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        key = (link_names[0], visual_names[0])
        expected_transparency = 0.5
        expected_return = {key: expected_transparency}
        with patch("deepsim.behaviours.transform.SetVisualTransparencyTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False
            transform = Transform(behaviour_mock, name)
            transform.set_transparency(expected_transparency)
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_transparencies.assert_not_called()
            assert transform._local_transparencies == {}

    def test_transparency(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.transparency = 0.5
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            assert transform.transparency == {key: 0.5}
            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=False)

    def test_transparency_setter(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        key = (link_names[0], visual_names[0])
        expected_transparency = 0.5
        expected_return = {key: expected_transparency}
        with patch("deepsim.behaviours.transform.SetVisualTransparencyTracker") as tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.transparency = expected_transparency
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_transparencies.assert_called_once_with(
                link_names=[self.link_name_format.format(myself())],
                visual_names=[self.visual_name],
                transparencies=[expected_transparency],
                blocking=False)
            assert transform._local_transparencies == expected_return

    def test_get_visible(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.visible = False
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            assert transform.get_visible() == {key: False}
            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=False)

    def test_get_visible_blocking(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.visible = False
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            assert transform.get_visible(blocking=True) == {key: False}
            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=True)

    def test_get_visible_no_model(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False
            visual_mock = MagicMock()
            visual_mock.visible = True
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            assert transform.get_visible() == {}
            tracker_mock.get_instance.return_value.get_visuals.assert_not_called()

    def test_get_visible_no_model_after_set(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False
            visual_mock = MagicMock()
            visual_mock.visible = False
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            transform.visible = True
            assert transform.get_visible() == {}
            tracker_mock.get_instance.return_value.get_visuals.assert_not_called()

    def test_set_visible(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        key = (link_names[0], visual_names[0])
        expected_visible = False
        expected_return = {key: expected_visible}
        with patch("deepsim.behaviours.transform.SetVisualVisibleTracker") as tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.set_visible(expected_visible)
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_visibles.assert_called_once_with(
                link_names=[self.link_name_format.format(myself())],
                visual_names=[self.visual_name],
                visibles=[expected_visible],
                blocking=False)
            assert transform._local_visibles == expected_return

    def test_set_visible_blocking(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        key = (link_names[0], visual_names[0])
        expected_visible = False
        expected_return = {key: expected_visible}
        with patch("deepsim.behaviours.transform.SetVisualVisibleTracker") as tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.set_visible(expected_visible,
                                  blocking=True)
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_visibles.assert_called_once_with(
                link_names=[self.link_name_format.format(myself())],
                visual_names=[self.visual_name],
                visibles=[expected_visible],
                blocking=True)
            assert transform._local_visibles == expected_return

    def test_set_visible_no_model(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        key = (link_names[0], visual_names[0])
        expected_visible = False
        expected_return = {key: expected_visible}
        with patch("deepsim.behaviours.transform.SetVisualVisibleTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False
            transform = Transform(behaviour_mock, name)
            transform.set_visible(expected_visible)
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_visibles.assert_not_called()
            assert transform._local_visibles == {}

    def test_visible(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.visible = False
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}
            transform = Transform(behaviour_mock, name)
            assert transform.visible == {key: False}
            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=False)

    def test_visible_setter(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        key = (link_names[0], visual_names[0])
        expected_visible = False
        expected_return = {key: expected_visible}
        with patch("deepsim.behaviours.transform.SetVisualVisibleTracker") as tracker_mock:
            transform = Transform(behaviour_mock, name)
            transform.visible = expected_visible
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.set_visual_visibles.assert_called_once_with(
                link_names=[self.link_name_format.format(myself())],
                visual_names=[self.visual_name],
                visibles=[expected_visible],
                blocking=False)
            assert transform._local_visibles == expected_return

    def test_get_visual_names(self, service_proxy_wrapper_mock, ros_util_mock):
        expected_link_names = [self.link_name_format.format(myself())]
        expected_visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(expected_link_names,
                                                                                 expected_visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        transform = Transform(behaviour_mock, name)
        with patch("deepsim.behaviours.transform.ROSUtil") as ros_util_mock:
            link_names, visual_names = transform._get_visual_names()
            assert link_names == expected_link_names
            assert visual_names == expected_visual_names
            assert transform._link_names == expected_link_names
            assert transform._visual_names == expected_visual_names

            _, _ = transform._get_visual_names()
            get_model_prop_mock = service_proxy_wrapper_mock(GazeboServiceName.GET_MODEL_PROPERTIES, None)
            req = GetModelPropertiesRequest()
            req.model_name = name
            get_model_prop_mock.assert_called_once_with(req)

    def test_get_visual_names_no_model(self, service_proxy_wrapper_mock, ros_util_mock):
        expected_link_names = [self.link_name_format.format(myself())]
        expected_visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(expected_link_names,
                                                                                 expected_visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        with patch("deepsim.behaviours.transform.ROSUtil") as ros_util_mock:
            ros_util_mock.is_model_spawned.return_value = False
            transform = Transform(behaviour_mock, name)
            assert transform._link_names == []
            assert transform._visual_names == []
            link_names, visual_names = transform._get_visual_names()
            assert link_names == []
            assert visual_names == []
            assert transform._link_names == []
            assert transform._visual_names == []
            get_model_prop_mock = service_proxy_wrapper_mock(GazeboServiceName.GET_MODEL_PROPERTIES, None)
            get_model_prop_mock.assert_not_called()

    def test_get_visuals(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            visual_mock = MagicMock()
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}

            expected_visuals = {key: visual_mock}

            transform = Transform(behaviour_mock, name)
            visuals = transform.get_visuals()

            assert expected_visuals == visuals
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock.get_instance.return_value.get_visuals.assert_called_once_with(link_names=link_names,
                                                                                       visual_names=visual_names,
                                                                                       blocking=False)

    def test_get_visuals_no_model(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()
        key = (link_names[0], visual_names[0])
        with patch("deepsim.behaviours.transform.GetVisualTracker") as tracker_mock:
            ros_util_mock.is_model_spawned.return_value = False
            visual_mock = MagicMock()
            visual_mock.link_name = link_names[0]
            visual_mock.visual_name = visual_names[0]
            tracker_mock.get_instance.return_value.get_visuals.return_value = {key: visual_mock}

            expected_visuals = {}

            transform = Transform(behaviour_mock, name)
            visuals = transform.get_visuals()

            assert expected_visuals == visuals
            tracker_mock_obj = tracker_mock.get_instance.return_value
            tracker_mock_obj.get_visuals.assert_not_called()

    def test_find(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        transform = Transform(behaviour_mock, name)

        assert Transform.find(name) == transform
        assert Transform.find("nonexisting", "no_failure") == "no_failure"

    def test_delete(self, service_proxy_wrapper_mock, ros_util_mock):
        link_names = [self.link_name_format.format(myself())]
        visual_names = [self.visual_name]
        service_proxy_wrapper_mock.side_effect = self.service_proxy_mock_creator(link_names,
                                                                                 visual_names)
        behaviour_mock = MagicMock()
        name = myself()

        transform = Transform(behaviour_mock, name)
        Transform.delete(name)
        assert Transform.find(name, None) is None
        assert transform.behaviour is None
