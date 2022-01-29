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

from deepsim.sim_trackers.trackers.get_visual_tracker import GetVisualTracker
from deepsim.gazebo.constants import GazeboTopicName, GazeboServiceName, GeometryType
from deepsim.math.color import Color
from deepsim.math.vector3 import Vector3
from deepsim.math.pose import Pose
from deepsim.math.quaternion import Quaternion
from deepsim.math.visual import Visual
from deepsim.math.material import Material
from deepsim.exception import DeepSimException

from deepsim_msgs.srv import (
    GetVisuals, GetVisualsResponse,
    GetAllVisuals, GetAllVisualsResponse
)
from deepsim_msgs.msg import Visuals

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.sim_trackers.trackers.get_visual_tracker.ServiceProxyWrapper")
class GetVisualTrackerTest(TestCase):
    def setUp(self) -> None:
        get_visuals_mock = MagicMock()
        get_all_visuals_mock = MagicMock()

        def get_service_mock(service_name, service_type):
            if service_name == GazeboServiceName.GET_VISUALS:
                return get_visuals_mock
            elif service_name == GazeboServiceName.GET_ALL_VISUALS:
                return get_all_visuals_mock
            else:
                return Exception()

        self.get_visuals_mock = get_visuals_mock
        self.get_all_visuals_mock = get_all_visuals_mock
        self.get_service_mock = get_service_mock

    def test_initialize(self, service_proxy_wrapper_mock):
        _ = GetVisualTracker(is_singleton=False)
        service_proxy_wrapper_mock.assert_has_calls([
            call(GazeboServiceName.GET_VISUALS, GetVisuals),
            call(GazeboServiceName.GET_ALL_VISUALS, GetAllVisuals)
        ])

    def test_on_update_tracker(self, service_proxy_wrapper_mock):
        link_name1 = myself() + "1"
        visual_name1 = myself() + "1_visual"
        ambient1 = Color(0.1, 0.2, 0.3, 0.4)
        diffuse1 = Color(0.2, 0.3, 0.4, 0.5)
        specular1 = Color(0.3, 0.4, 0.5, 0.6)
        emissive1 = Color(0.4, 0.5, 0.6, 0.7)
        transparency1 = 0.3
        visible1 = True
        geometry_type1 = GeometryType.MESH
        mesh_geom_filename1 = myself() + "_mesh_geom_filename1"
        mesh_geom_scale1 = Vector3(1.0, 2.0, 3.0)
        pose1 = Pose(position=Vector3(2.0, 3.0, 4.0),
                     orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        link_name2 = myself() + "2"
        visual_name2 = myself() + "2_visual"
        ambient2 = Color(0.2, 0.2, 0.3, 0.4)
        diffuse2 = Color(0.3, 0.3, 0.4, 0.5)
        specular2 = Color(0.4, 0.4, 0.5, 0.6)
        emissive2 = Color(0.5, 0.5, 0.6, 0.7)
        transparency2 = 0.5
        visible2 = True
        geometry_type2 = GeometryType.BOX
        mesh_geom_filename2 = myself() + "_mesh_geom_filename2"
        mesh_geom_scale2 = Vector3(2.0, 2.0, 3.0)
        pose2 = Pose(position=Vector3(3.0, 3.0, 4.0),
                     orientation=Quaternion(4.0, 4.0, 5.0, 6.0))

        expected_visual1 = Visual(link_name=link_name1,
                                  visual_name=visual_name1,
                                  material=Material(ambient=ambient1,
                                                    diffuse=diffuse1,
                                                    specular=specular1,
                                                    emissive=emissive1),
                                  transparency=transparency1,
                                  visible=visible1,
                                  geometry_type=geometry_type1,
                                  mesh_geom_filename=mesh_geom_filename1,
                                  mesh_geom_scale=mesh_geom_scale1,
                                  pose=pose1)

        expected_visual2 = Visual(link_name=link_name2,
                                  visual_name=visual_name2,
                                  material=Material(ambient=ambient2,
                                                    diffuse=diffuse2,
                                                    specular=specular2,
                                                    emissive=emissive2),
                                  transparency=transparency2,
                                  visible=visible2,
                                  geometry_type=geometry_type2,
                                  mesh_geom_filename=mesh_geom_filename2,
                                  mesh_geom_scale=mesh_geom_scale2,
                                  pose=pose2)

        res = GetAllVisualsResponse()
        res.success = True
        res.status_message = ''
        res.visuals = [expected_visual1.to_ros(),
                       expected_visual2.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_visuals_mock.return_value = res

        tracker = GetVisualTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        assert (link_name1, visual_name1) in tracker._visual_map
        assert (link_name2, visual_name2) in tracker._visual_map
        assert expected_visual1 == tracker._visual_map[(link_name1, visual_name1)]
        assert expected_visual2 == tracker._visual_map[(link_name2, visual_name2)]

    def test_get_visual(self, service_proxy_wrapper_mock):
        link_name1 = myself() + "1"
        visual_name1 = myself() + "1_visual"
        ambient1 = Color(0.1, 0.2, 0.3, 0.4)
        diffuse1 = Color(0.2, 0.3, 0.4, 0.5)
        specular1 = Color(0.3, 0.4, 0.5, 0.6)
        emissive1 = Color(0.4, 0.5, 0.6, 0.7)
        transparency1 = 0.3
        visible1 = True
        geometry_type1 = GeometryType.MESH
        mesh_geom_filename1 = myself() + "_mesh_geom_filename1"
        mesh_geom_scale1 = Vector3(1.0, 2.0, 3.0)
        pose1 = Pose(position=Vector3(2.0, 3.0, 4.0),
                     orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        link_name2 = myself() + "2"
        visual_name2 = myself() + "2_visual"
        ambient2 = Color(0.2, 0.2, 0.3, 0.4)
        diffuse2 = Color(0.3, 0.3, 0.4, 0.5)
        specular2 = Color(0.4, 0.4, 0.5, 0.6)
        emissive2 = Color(0.5, 0.5, 0.6, 0.7)
        transparency2 = 0.5
        visible2 = True
        geometry_type2 = GeometryType.BOX
        mesh_geom_filename2 = myself() + "_mesh_geom_filename2"
        mesh_geom_scale2 = Vector3(2.0, 2.0, 3.0)
        pose2 = Pose(position=Vector3(3.0, 3.0, 4.0),
                     orientation=Quaternion(4.0, 4.0, 5.0, 6.0))

        expected_visual1 = Visual(link_name=link_name1,
                                  visual_name=visual_name1,
                                  material=Material(ambient=ambient1,
                                                    diffuse=diffuse1,
                                                    specular=specular1,
                                                    emissive=emissive1),
                                  transparency=transparency1,
                                  visible=visible1,
                                  geometry_type=geometry_type1,
                                  mesh_geom_filename=mesh_geom_filename1,
                                  mesh_geom_scale=mesh_geom_scale1,
                                  pose=pose1)

        expected_visual2 = Visual(link_name=link_name2,
                                  visual_name=visual_name2,
                                  material=Material(ambient=ambient2,
                                                    diffuse=diffuse2,
                                                    specular=specular2,
                                                    emissive=emissive2),
                                  transparency=transparency2,
                                  visible=visible2,
                                  geometry_type=geometry_type2,
                                  mesh_geom_filename=mesh_geom_filename2,
                                  mesh_geom_scale=mesh_geom_scale2,
                                  pose=pose2)

        res = GetAllVisualsResponse()
        res.success = True
        res.status_message = ''
        res.visuals = [expected_visual1.to_ros(),
                       expected_visual2.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_visuals_mock.return_value = res

        tracker = GetVisualTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        visual1 = tracker.get_visual(link_name1, visual_name1)
        visual2 = tracker.get_visual(link_name2, visual_name2)

        assert visual1 == expected_visual1
        assert visual2 == expected_visual2

        # Check returned visual is a copy
        assert tracker._visual_map[(link_name1, visual_name1)] == visual1
        assert tracker._visual_map[(link_name1, visual_name1)] is not visual1
        assert tracker._visual_map[(link_name2, visual_name2)] == visual2
        assert tracker._visual_map[(link_name2, visual_name2)] is not visual2
        self.get_visuals_mock.assert_not_called()

    def test_get_visual_blocking(self, service_proxy_wrapper_mock):
        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)
        transparency = 0.3
        visible = True
        geometry_type = GeometryType.MESH
        mesh_geom_filename = myself() + "_mesh_geom_filename1"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        expected_visual = Visual(link_name=link_name,
                                 visual_name=visual_name,
                                 material=Material(ambient=ambient,
                                                   diffuse=diffuse,
                                                   specular=specular,
                                                   emissive=emissive),
                                 transparency=transparency,
                                 visible=visible,
                                 geometry_type=geometry_type,
                                 mesh_geom_filename=mesh_geom_filename,
                                 mesh_geom_scale=mesh_geom_scale,
                                 pose=pose)

        res = GetVisualsResponse()
        res.status = [True]
        res.success = True
        res.visuals = [expected_visual.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_visuals_mock.return_value = res

        tracker = GetVisualTracker(is_singleton=False)
        visual = tracker.get_visual(link_name, visual_name, blocking=True)
        self.get_visuals_mock.assert_called_once_with([link_name], [visual_name])
        assert visual == expected_visual

    def test_get_visual_missing_in_dict(self, service_proxy_wrapper_mock):
        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)
        transparency = 0.3
        visible = True
        geometry_type = GeometryType.MESH
        mesh_geom_filename = myself() + "_mesh_geom_filename1"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        expected_visual = Visual(link_name=link_name,
                                 visual_name=visual_name,
                                 material=Material(ambient=ambient,
                                                   diffuse=diffuse,
                                                   specular=specular,
                                                   emissive=emissive),
                                 transparency=transparency,
                                 visible=visible,
                                 geometry_type=geometry_type,
                                 mesh_geom_filename=mesh_geom_filename,
                                 mesh_geom_scale=mesh_geom_scale,
                                 pose=pose)

        res = GetVisualsResponse()
        res.status = [True]
        res.success = True
        res.visuals = [expected_visual.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_visuals_mock.return_value = res

        tracker = GetVisualTracker(is_singleton=False)
        visual = tracker.get_visual(link_name, visual_name)
        self.get_visuals_mock.assert_called_once_with([link_name], [visual_name])
        assert visual == expected_visual

    def test_get_visual_missing_and_failed_to_retrieve(self, service_proxy_wrapper_mock):
        link_name = myself()
        visual_name = myself() + "_visual"

        res = GetVisualsResponse()
        res.status = [False]
        res.success = False
        res.visuals = [None]
        res.messages = ["Failed"]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_visuals_mock.return_value = res

        tracker = GetVisualTracker(is_singleton=False)
        with self.assertRaises(DeepSimException):
            tracker.get_visual(link_name, visual_name)

        res.status = [False]
        res.success = True

        with self.assertRaises(DeepSimException):
            tracker.get_visual(link_name, visual_name)

        res.status = [True]
        res.success = False

        with self.assertRaises(DeepSimException):
            tracker.get_visual(link_name, visual_name)

        self.get_visuals_mock.has_calls(
            call([link_name], [visual_name]),
            call([link_name], [visual_name]),
            call([link_name], [visual_name])
        )

    def test_get_visuals(self, service_proxy_wrapper_mock):
        link_name1 = myself() + "1"
        visual_name1 = myself() + "1_visual"
        ambient1 = Color(0.1, 0.2, 0.3, 0.4)
        diffuse1 = Color(0.2, 0.3, 0.4, 0.5)
        specular1 = Color(0.3, 0.4, 0.5, 0.6)
        emissive1 = Color(0.4, 0.5, 0.6, 0.7)
        transparency1 = 0.3
        visible1 = True
        geometry_type1 = GeometryType.MESH
        mesh_geom_filename1 = myself() + "_mesh_geom_filename1"
        mesh_geom_scale1 = Vector3(1.0, 2.0, 3.0)
        pose1 = Pose(position=Vector3(2.0, 3.0, 4.0),
                     orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        link_name2 = myself() + "2"
        visual_name2 = myself() + "2_visual"
        ambient2 = Color(0.2, 0.2, 0.3, 0.4)
        diffuse2 = Color(0.3, 0.3, 0.4, 0.5)
        specular2 = Color(0.4, 0.4, 0.5, 0.6)
        emissive2 = Color(0.5, 0.5, 0.6, 0.7)
        transparency2 = 0.5
        visible2 = True
        geometry_type2 = GeometryType.BOX
        mesh_geom_filename2 = myself() + "_mesh_geom_filename2"
        mesh_geom_scale2 = Vector3(2.0, 2.0, 3.0)
        pose2 = Pose(position=Vector3(3.0, 3.0, 4.0),
                     orientation=Quaternion(4.0, 4.0, 5.0, 6.0))

        expected_visual1 = Visual(link_name=link_name1,
                                  visual_name=visual_name1,
                                  material=Material(ambient=ambient1,
                                                    diffuse=diffuse1,
                                                    specular=specular1,
                                                    emissive=emissive1),
                                  transparency=transparency1,
                                  visible=visible1,
                                  geometry_type=geometry_type1,
                                  mesh_geom_filename=mesh_geom_filename1,
                                  mesh_geom_scale=mesh_geom_scale1,
                                  pose=pose1)

        expected_visual2 = Visual(link_name=link_name2,
                                  visual_name=visual_name2,
                                  material=Material(ambient=ambient2,
                                                    diffuse=diffuse2,
                                                    specular=specular2,
                                                    emissive=emissive2),
                                  transparency=transparency2,
                                  visible=visible2,
                                  geometry_type=geometry_type2,
                                  mesh_geom_filename=mesh_geom_filename2,
                                  mesh_geom_scale=mesh_geom_scale2,
                                  pose=pose2)

        res = GetAllVisualsResponse()
        res.success = True
        res.status_message = ''
        res.visuals = [expected_visual1.to_ros(),
                       expected_visual2.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_visuals_mock.return_value = res

        tracker = GetVisualTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        visual_map = tracker.get_visuals([link_name1, link_name2],
                                         [visual_name1, visual_name2])

        key1 = (link_name1, visual_name1)
        key2 = (link_name2, visual_name2)
        expected_visuals = {key1: expected_visual1,
                            key2: expected_visual2}
        assert visual_map == expected_visuals

        self.get_visuals_mock.assert_not_called()

    def test_get_visuals_blocking(self, service_proxy_wrapper_mock):
        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)
        transparency = 0.3
        visible = True
        geometry_type = GeometryType.MESH
        mesh_geom_filename = myself() + "_mesh_geom_filename1"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        expected_visual = Visual(link_name=link_name,
                                 visual_name=visual_name,
                                 material=Material(ambient=ambient,
                                                   diffuse=diffuse,
                                                   specular=specular,
                                                   emissive=emissive),
                                 transparency=transparency,
                                 visible=visible,
                                 geometry_type=geometry_type,
                                 mesh_geom_filename=mesh_geom_filename,
                                 mesh_geom_scale=mesh_geom_scale,
                                 pose=pose)

        key = (link_name, visual_name)
        expected_return = {key: expected_visual}

        res = GetVisualsResponse()
        res.status = [True]
        res.success = True
        res.visuals = [expected_visual.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_visuals_mock.return_value = res

        tracker = GetVisualTracker(is_singleton=False)
        visuals = tracker.get_visuals([link_name], [visual_name], blocking=True)

        self.get_visuals_mock.assert_called_once_with([link_name], [visual_name])
        assert visuals == expected_return

    def test_get_visuals_missing_in_dict(self, service_proxy_wrapper_mock):
        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 0.4)
        diffuse = Color(0.2, 0.3, 0.4, 0.5)
        specular = Color(0.3, 0.4, 0.5, 0.6)
        emissive = Color(0.4, 0.5, 0.6, 0.7)
        transparency = 0.3
        visible = True
        geometry_type = GeometryType.MESH
        mesh_geom_filename = myself() + "_mesh_geom_filename1"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        expected_visual = Visual(link_name=link_name,
                                 visual_name=visual_name,
                                 material=Material(ambient=ambient,
                                                   diffuse=diffuse,
                                                   specular=specular,
                                                   emissive=emissive),
                                 transparency=transparency,
                                 visible=visible,
                                 geometry_type=geometry_type,
                                 mesh_geom_filename=mesh_geom_filename,
                                 mesh_geom_scale=mesh_geom_scale,
                                 pose=pose)
        key = (link_name, visual_name)
        expected_return = {key: expected_visual}

        res = GetVisualsResponse()
        res.status = [True]
        res.success = True
        res.visuals = [expected_visual.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_visuals_mock.return_value = res

        tracker = GetVisualTracker(is_singleton=False)
        visuals = tracker.get_visuals([link_name], [visual_name])
        self.get_visuals_mock.assert_called_once_with([link_name], [visual_name])
        assert visuals == expected_return

    def test_get_visuals_missing_and_failed_to_retrieve(self, service_proxy_wrapper_mock):
        link_name = myself()
        visual_name = myself() + "_visual"

        res = GetVisualsResponse()
        res.status = [False]
        res.success = False
        res.visuals = [None]
        res.messages = ["Failed"]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_visuals_mock.return_value = res

        tracker = GetVisualTracker(is_singleton=False)
        with self.assertRaises(DeepSimException):
            tracker.get_visuals([link_name], [visual_name])

        res.status = [False]
        res.success = True

        assert tracker.get_visuals([link_name], [visual_name]) == {(link_name, visual_name): None}

        res.status = [True]
        res.success = False

        with self.assertRaises(DeepSimException):
            tracker.get_visuals([link_name], [visual_name])

        self.get_visuals_mock.has_calls(
            call([link_name], [visual_name]),
            call([link_name], [visual_name]),
            call([link_name], [visual_name])
        )

    def test_get_visuals_unmatched_length(self, service_proxy_wrapper_mock):
        tracker = GetVisualTracker(is_singleton=False)

        link_name = myself()
        with self.assertRaises(ValueError):
            tracker.get_visuals([link_name], [])

    def test_set_material(self, service_proxy_wrapper_mock):
        tracker = GetVisualTracker(is_singleton=False)
        link_name = myself()
        visual_name = myself() + '_visual'

        key = (link_name, visual_name)
        tracker._visual_map[key] = Visual()
        expected_material = Material(ambient=Color(1.0, 2.0, 3.0))

        tracker.set_material(link_name=link_name,
                             visual_name=visual_name,
                             material=expected_material)
        assert expected_material == tracker.get_visual(link_name=link_name,
                                                       visual_name=visual_name).material

    def test_set_material_visual_not_in_map(self, service_proxy_wrapper_mock):
        tracker = GetVisualTracker(is_singleton=False)
        link_name = myself()
        visual_name = myself() + '_visual'

        key = (link_name, visual_name)
        new_material = Material(ambient=Color(1.0, 2.0, 3.0))

        tracker.set_material(link_name=link_name,
                             visual_name=visual_name,
                             material=new_material)
        assert key not in tracker._visual_map

    def test_set_transparency(self, service_proxy_wrapper_mock):
        tracker = GetVisualTracker(is_singleton=False)
        link_name = myself()
        visual_name = myself() + '_visual'

        key = (link_name, visual_name)
        tracker._visual_map[key] = Visual()

        expected_transparency = 0.5
        tracker.set_transparency(link_name=link_name,
                                 visual_name=visual_name,
                                 transparency=expected_transparency)
        assert expected_transparency == tracker.get_visual(link_name=link_name,
                                                           visual_name=visual_name).transparency

    def test_set_transparency_visual_not_in_map(self, service_proxy_wrapper_mock):
        tracker = GetVisualTracker(is_singleton=False)
        link_name = myself()
        visual_name = myself() + '_visual'

        key = (link_name, visual_name)

        new_transparency = 0.5
        tracker.set_transparency(link_name=link_name,
                                 visual_name=visual_name,
                                 transparency=new_transparency)
        assert key not in tracker._visual_map

    def test_set_visible(self, service_proxy_wrapper_mock):
        tracker = GetVisualTracker(is_singleton=False)
        link_name = myself()
        visual_name = myself() + '_visual'

        key = (link_name, visual_name)
        tracker._visual_map[key] = Visual()

        expected_visible = False
        tracker.set_visible(link_name=link_name,
                            visual_name=visual_name,
                            visible=expected_visible)
        assert expected_visible == tracker.get_visual(link_name=link_name,
                                                      visual_name=visual_name).visible

    def test_set_visible_visual_not_in_map(self, service_proxy_wrapper_mock):
        tracker = GetVisualTracker(is_singleton=False)
        link_name = myself()
        visual_name = myself() + '_visual'

        key = (link_name, visual_name)

        new_visible = False
        tracker.set_visible(link_name=link_name,
                            visual_name=visual_name,
                            visible=new_visible)
        assert key not in tracker._visual_map
