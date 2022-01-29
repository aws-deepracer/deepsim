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
"""A class for visual."""
from typing import Optional, Union
from deepsim.math.color import Color
from deepsim.math.vector3 import Vector3
from deepsim.math.pose import Pose
from deepsim.math.material import Material
from deepsim.gazebo.constants import GeometryType

from deepsim_msgs.msg import Visual as ROSVisual


class Visual:
    """
    Visual class
    """

    def __init__(self,
                 link_name: Optional[str] = None,
                 visual_name: Optional[str] = None,
                 material: Optional[Material] = None,
                 transparency: float = 0.0,
                 visible: bool = True,
                 geometry_type: Union[int, GeometryType] = GeometryType.EMPTY,
                 mesh_geom_filename: Optional[str] = None,
                 mesh_geom_scale: Optional[Vector3] = None,
                 pose: Optional[Pose] = None) -> None:
        """
        Initialize Visual class

        Args:
            link_name (Optional[str]): link name
            visual_name (Optional[str]): visual name
            material (Optional[Material]): material
            transparency (float): transparency [0.0 - 1.0]
            visible (bool): the flag whether visible or not
            geometry_type (Union[int, GeometryType]): the geometry type
            mesh_geom_filename (Optional[str]): mesh geometry filename
            mesh_geom_scale (Optional[Vector3]): the mesh scale in vector3 format.
            pose (Optional[Pose]): the pose of visual
        """
        self._link_name = link_name
        self._visual_name = visual_name
        self._material = material.copy() if material else Material()

        self._transparency = transparency
        self._visible = visible
        self._geometry_type = geometry_type
        if isinstance(self._geometry_type, int):
            self._geometry_type = GeometryType(self._geometry_type)
        self._mesh_geom_filename = mesh_geom_filename
        self._mesh_geom_scale = mesh_geom_scale.copy() if mesh_geom_scale else Vector3.one()
        self._pose = pose.copy() if pose else Pose()

    @property
    def link_name(self) -> str:
        """
        Returns the link name

        Returns:
            str: link name
        """
        return self._link_name

    @link_name.setter
    def link_name(self, value: str) -> None:
        """
        Set link name

        Args:
            value (str): link name
        """
        self._link_name = value

    @property
    def visual_name(self) -> str:
        """
        Returns the visual name

        Returns:
            str: visual name
        """
        return self._visual_name

    @visual_name.setter
    def visual_name(self, value: str) -> None:
        """
        Set visual name

        Args:
            value (str): visual name
        """
        self._visual_name = value

    @property
    def material(self) -> Material:
        """
        Returns the copy of material

        Returns:
            Material: the copy of material
        """
        return self._material.copy()

    @material.setter
    def material(self, value: Material) -> None:
        """
        Set the material

        Args:
            value (Material): the material
        """
        self._material = value.copy()

    @property
    def transparency(self) -> float:
        """
        Returns the transparency [0.0 - 1.0]
        - 0.0 is full transparency and 1.0 is opaque.

        Returns:
            float: the transparency value
        """
        return self._transparency

    @transparency.setter
    def transparency(self, value: float) -> None:
        """
        Set the transparency

        Args:
            value (float): the transparency
        """
        self._transparency = value

    @property
    def visible(self) -> bool:
        """
        Returns the flag whether visible or not.

        Returns:
            bool: the flag whether visible or not.
        """
        return self._visible

    @visible.setter
    def visible(self, value: bool) -> None:
        """
        Set the visible

        Args:
            value (bool): the visible
        """
        self._visible = value

    @property
    def geometry_type(self) -> GeometryType:
        """
        Returns the geometry type

        Returns:
            int: the geometry type
        """
        return self._geometry_type

    @geometry_type.setter
    def geometry_type(self, value: Union[int, GeometryType]) -> None:
        """
        Set the geometry_type

        Args:
            value (GeometryType): the geometry_type
        """
        self._geometry_type = value
        if isinstance(self._geometry_type, int):
            self._geometry_type = GeometryType(self._geometry_type)

    @property
    def mesh_geom_filename(self) -> str:
        """
        Returns the mesh geometry filename

        Returns:
            str: mesh geometry filename
        """
        return self._mesh_geom_filename

    @mesh_geom_filename.setter
    def mesh_geom_filename(self, value: str) -> None:
        """
        Set mesh geometry filename

        Args:
            value (str):  mesh geometry filename
        """
        self._mesh_geom_filename = value

    @property
    def mesh_geom_scale(self) -> Vector3:
        """
        Returns the mesh geometry scale in Vector3

        Returns:
            Vector3: the mesh geometry scale in Vector3
        """
        return self._mesh_geom_scale.copy()

    @mesh_geom_scale.setter
    def mesh_geom_scale(self, value: Vector3) -> None:
        """
        Set the mesh geometry scale in Vector3

        Args:
            value (Vector3): the mesh geometry scale in Vector3
        """
        self._mesh_geom_scale = value.copy()

    @property
    def pose(self) -> Pose:
        """
        Returns the pose of the visual

        Returns:
            Pose: the pose of the visual
        """
        return self._pose.copy()

    @pose.setter
    def pose(self, value: Pose) -> None:
        """
        Set the pose of the visual

        Args:
            value (Pose): the pose of the visual
        """
        self._pose = value.copy()

    def to_ros(self) -> ROSVisual:
        ros_visual = ROSVisual()
        ros_visual.link_name = self.link_name
        ros_visual.visual_name = self.visual_name
        ros_visual.ambient = self.material.ambient.to_ros()
        ros_visual.diffuse = self.material.diffuse.to_ros()
        ros_visual.specular = self.material.specular.to_ros()
        ros_visual.emissive = self.material.emissive.to_ros()
        ros_visual.transparency = self.transparency
        ros_visual.visible = self.visible
        ros_visual.geometry_type = self.geometry_type.value
        ros_visual.mesh_geom_filename = self.mesh_geom_filename
        ros_visual.mesh_geom_scale = self.mesh_geom_scale.to_ros()
        ros_visual.pose = self._pose.to_ros()
        return ros_visual

    @staticmethod
    def from_ros(value: ROSVisual) -> 'Visual':
        """
        Returns new Visual object created from ROS GetVisualResponse

        Args:
            value (deepsim_msgs.msg.ROSVisual): ROS Visual

        Returns:
            Pose: new Visual object created from ROS Visual
        """
        return Visual(link_name=value.link_name,
                      visual_name=value.visual_name,
                      material=Material(ambient=Color.from_ros(value.ambient),
                                        diffuse=Color.from_ros(value.diffuse),
                                        specular=Color.from_ros(value.specular),
                                        emissive=Color.from_ros(value.emissive)),
                      transparency=value.transparency,
                      visible=value.visible,
                      geometry_type=value.geometry_type,
                      mesh_geom_filename=value.mesh_geom_filename,
                      mesh_geom_scale=Vector3.from_ros(value.mesh_geom_scale),
                      pose=Pose.from_ros(value.pose))

    def copy(self) -> 'Visual':
        """
        Returns a copy.

        Returns:
            Visual: the copied visual
        """
        return Visual(link_name=self._link_name,
                      visual_name=self._visual_name,
                      material=self._material,
                      transparency=self._transparency,
                      visible=self._visible,
                      geometry_type=self._geometry_type,
                      mesh_geom_filename=self._mesh_geom_filename,
                      mesh_geom_scale=self._mesh_geom_scale,
                      pose=self._pose)

    def __eq__(self, other: 'Visual') -> bool:
        """
        Check whether given visual is equal to self.

        Args:
            other (Visual): other to compare

        Returns:
            bool: True if the differences of all components are within epsilon, Otherwise False.
        """
        return (self._link_name == other._link_name
                and self._visual_name == other._visual_name
                and self._material == other._material
                and self._transparency == other._transparency
                and self._visible == other._visible
                and self._geometry_type == other._geometry_type
                and self._mesh_geom_filename == other._mesh_geom_filename
                and self._mesh_geom_scale == other._mesh_geom_scale
                and self._pose == other._pose)

    def __ne__(self, other: 'Visual') -> bool:
        """
        Inequality of visuals is inequality of any components.

        Args:
            other (Visual): other to compare

        Returns:
            bool: False if there are the differences of any components, Otherwise True.
        """
        return not self.__eq__(other)

    def __str__(self) -> str:
        """
        String representation of a visual

        Returns:
            str: String representation of a visual
        """
        repr_str = "("
        repr_str += "link_name={}".format(repr(self._link_name))
        repr_str += ", visual_name={}".format(repr(self._visual_name))
        repr_str += ", material={}".format(repr(self._material))
        repr_str += ", transparency={}".format(repr(self._transparency))
        repr_str += ", visible={}".format(repr(self._visible))
        repr_str += ", geometry_type={}".format(repr(self._geometry_type.name))
        repr_str += ", mesh_geom_filename={}".format(repr(self._mesh_geom_filename))
        repr_str += ", mesh_geom_scale={}".format(repr(self._mesh_geom_scale))
        repr_str += ", pose={})".format(repr(self._pose))
        return repr_str

    def __repr__(self) -> str:
        """
        String representation including class

        Returns:
            str: String representation including class
        """
        return "Visual" + str(self)
