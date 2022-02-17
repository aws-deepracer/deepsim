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

from deepsim.core.visual import Visual
from deepsim.core.color import Color
from deepsim.core.vector3 import Vector3
from deepsim.core.pose import Pose
from deepsim.core.quaternion import Quaternion
from deepsim.core.material import Material
from deepsim.gazebo.constants import GeometryType

from deepsim_msgs.msg import Visual as ROSVisual


myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class VisualTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        visual = Visual()
        assert visual.link_name is None
        assert visual.visual_name is None
        assert visual.material == Material()
        assert visual.transparency == 0.0
        assert visual.visible
        assert visual.geometry_type == GeometryType.EMPTY
        assert visual.mesh_geom_filename is None
        assert visual.mesh_geom_scale == Vector3.one()
        assert visual.pose == Pose()

        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 1.0)
        diffuse = Color(0.1, 0.2, 0.3, 1.0)
        specular = Color(0.1, 0.2, 0.3, 1.0)
        emissive = Color(0.1, 0.2, 0.3, 1.0)
        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)
        transparency = 0.5
        visible = False
        geometry_type = GeometryType.BOX
        mesh_geom_filename = myself() + "_filename"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))
        
        visual = Visual(link_name=link_name,
                        visual_name=visual_name,
                        material=material,
                        transparency=transparency,
                        visible=visible,
                        geometry_type=geometry_type,
                        mesh_geom_filename=mesh_geom_filename,
                        mesh_geom_scale=mesh_geom_scale,
                        pose=pose)
        assert visual.link_name == link_name
        assert visual.visual_name == visual_name
        assert visual.material == material
        assert visual.material is not material
        assert visual.transparency == transparency
        assert visual.visible == visible
        assert visual.geometry_type == geometry_type
        assert visual.mesh_geom_filename == mesh_geom_filename
        assert visual.mesh_geom_scale == mesh_geom_scale
        assert visual.mesh_geom_scale is not mesh_geom_scale
        assert visual.pose == pose
        assert visual.pose is not pose
    
    def test_setters(self):
        visual = Visual()
        assert visual.link_name is None
        assert visual.visual_name is None
        assert visual.material == Material()
        assert visual.transparency == 0.0
        assert visual.visible
        assert visual.geometry_type == GeometryType.EMPTY
        assert visual.mesh_geom_filename is None
        assert visual.mesh_geom_scale == Vector3.one()
        assert visual.pose == Pose()

        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 1.0)
        diffuse = Color(0.1, 0.2, 0.3, 1.0)
        specular = Color(0.1, 0.2, 0.3, 1.0)
        emissive = Color(0.1, 0.2, 0.3, 1.0)
        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)
        transparency = 0.5
        visible = False
        geometry_type = GeometryType.MESH
        mesh_geom_filename = myself() + "_filename"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        visual.link_name = link_name
        visual.visual_name = visual_name
        visual.material = material
        visual.transparency = transparency
        visual.visible = visible
        visual.geometry_type = geometry_type
        visual.mesh_geom_filename = mesh_geom_filename
        visual.mesh_geom_scale = mesh_geom_scale
        visual.pose = pose

        assert visual.link_name == link_name
        assert visual.visual_name == visual_name
        assert visual.material == material
        assert visual.material is not material
        assert visual.transparency == transparency
        assert visual.visible == visible
        assert visual.geometry_type == geometry_type
        assert visual.mesh_geom_filename == mesh_geom_filename
        assert visual.mesh_geom_scale == mesh_geom_scale
        assert visual.mesh_geom_scale is not mesh_geom_scale
        assert visual.pose == pose
        assert visual.pose is not pose

    def test_to_ros(self):
        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 1.0)
        diffuse = Color(0.1, 0.2, 0.3, 1.0)
        specular = Color(0.1, 0.2, 0.3, 1.0)
        emissive = Color(0.1, 0.2, 0.3, 1.0)
        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)
        transparency = 0.5
        visible = False
        geometry_type = GeometryType.MESH
        mesh_geom_filename = myself() + "_filename"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        ros_visual = ROSVisual()
        ros_visual.link_name = link_name
        ros_visual.visual_name = visual_name
        ros_visual.ambient = ambient.to_ros()
        ros_visual.diffuse = diffuse.to_ros()
        ros_visual.specular = specular.to_ros()
        ros_visual.emissive = emissive.to_ros()
        ros_visual.transparency = transparency
        ros_visual.visible = visible
        ros_visual.geometry_type = geometry_type.value
        ros_visual.mesh_geom_filename = mesh_geom_filename
        ros_visual.mesh_geom_scale = mesh_geom_scale.to_ros()
        ros_visual.pose = pose.to_ros()

        visual = Visual(link_name=link_name,
                        visual_name=visual_name,
                        material=material,
                        transparency=transparency,
                        visible=visible,
                        geometry_type=geometry_type,
                        mesh_geom_filename=mesh_geom_filename,
                        mesh_geom_scale=mesh_geom_scale,
                        pose=pose)

        assert visual.to_ros() == ros_visual

    def test_from_ros(self):
        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 1.0)
        diffuse = Color(0.1, 0.2, 0.3, 1.0)
        specular = Color(0.1, 0.2, 0.3, 1.0)
        emissive = Color(0.1, 0.2, 0.3, 1.0)
        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)
        transparency = 0.5
        visible = False
        geometry_type = GeometryType.MESH
        mesh_geom_filename = myself() + "_filename"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        ros_visual = ROSVisual()
        ros_visual.link_name = link_name
        ros_visual.visual_name = visual_name
        ros_visual.ambient = ambient.to_ros()
        ros_visual.diffuse = diffuse.to_ros()
        ros_visual.specular = specular.to_ros()
        ros_visual.emissive = emissive.to_ros()
        ros_visual.transparency = transparency
        ros_visual.visible = visible
        ros_visual.geometry_type = geometry_type
        ros_visual.mesh_geom_filename = mesh_geom_filename
        ros_visual.mesh_geom_scale = mesh_geom_scale.to_ros()
        ros_visual.pose = pose.to_ros()

        expected_visual = Visual(link_name=link_name,
                                 visual_name=visual_name,
                                 material=material,
                                 transparency=transparency,
                                 visible=visible,
                                 geometry_type=geometry_type,
                                 mesh_geom_filename=mesh_geom_filename,
                                 mesh_geom_scale=mesh_geom_scale,
                                 pose=pose)

        assert Visual.from_ros(ros_visual) == expected_visual

    def test_copy(self):
        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 1.0)
        diffuse = Color(0.1, 0.2, 0.3, 1.0)
        specular = Color(0.1, 0.2, 0.3, 1.0)
        emissive = Color(0.1, 0.2, 0.3, 1.0)
        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)
        transparency = 0.5
        visible = False
        geometry_type = GeometryType.BOX
        mesh_geom_filename = myself() + "_filename"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        visual = Visual(link_name=link_name,
                        visual_name=visual_name,
                        material=material,
                        transparency=transparency,
                        visible=visible,
                        geometry_type=geometry_type,
                        mesh_geom_filename=mesh_geom_filename,
                        mesh_geom_scale=mesh_geom_scale,
                        pose=pose)

        visual_copy = visual.copy()
        assert visual_copy == visual

        assert visual.link_name == visual.link_name
        assert visual.visual_name == visual.visual_name
        assert visual.material == visual.material
        assert visual.material is not visual.material
        assert visual.transparency == visual.transparency
        assert visual.visible == visual.visible
        assert visual.geometry_type == visual.geometry_type
        assert visual.mesh_geom_filename == visual.mesh_geom_filename
        assert visual.mesh_geom_scale == visual.mesh_geom_scale
        assert visual.mesh_geom_scale is not visual.mesh_geom_scale
        assert visual.pose == visual.pose
        assert visual.pose is not visual.pose

    def test_eq(self):
        link_name = myself()
        visual_name = myself() + "_visual"
        ambient = Color(0.1, 0.2, 0.3, 1.0)
        diffuse = Color(0.1, 0.2, 0.3, 1.0)
        specular = Color(0.1, 0.2, 0.3, 1.0)
        emissive = Color(0.1, 0.2, 0.3, 1.0)
        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)
        transparency = 0.5
        visible = False
        geometry_type = GeometryType.SPHERE
        mesh_geom_filename = myself() + "_filename"
        mesh_geom_scale = Vector3(1.0, 2.0, 3.0)
        pose = Pose(position=Vector3(2.0, 3.0, 4.0),
                    orientation=Quaternion(3.0, 4.0, 5.0, 6.0))

        visual = Visual(link_name=link_name,
                        visual_name=visual_name,
                        material=material,
                        transparency=transparency,
                        visible=visible,
                        geometry_type=geometry_type,
                        mesh_geom_filename=mesh_geom_filename,
                        mesh_geom_scale=mesh_geom_scale,
                        pose=pose)

        visual2 = Visual(link_name=link_name,
                         visual_name=visual_name,
                         material=material,
                         transparency=transparency,
                         visible=visible,
                         geometry_type=geometry_type,
                         mesh_geom_filename=mesh_geom_filename,
                         mesh_geom_scale=mesh_geom_scale,
                         pose=pose)

        assert visual == visual2

        pose2 = Pose(position=Vector3(3.0, 3.0, 4.0),
                     orientation=Quaternion(4.0, 4.0, 5.0, 6.0))

        visual2 = Visual(link_name=link_name,
                         visual_name=visual_name,
                         material=material,
                         transparency=transparency,
                         visible=visible,
                         geometry_type=geometry_type,
                         mesh_geom_filename=mesh_geom_filename,
                         mesh_geom_scale=mesh_geom_scale,
                         pose=pose2)
        assert visual != visual2
