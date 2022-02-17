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
"""Module to contain Gazebo related constants"""
from enum import Enum
from deepsim.core.vector3 import Vector3


# Define Gazebo World default direction unit vectors
class GazeboWorld(Enum):
    """Gazebo World direction unit vectors"""
    FORWARD = (1.0, 0, 0)
    BACK = (-1.0, 0, 0)
    RIGHT = (0, -1.0, 0)
    LEFT = (0, 1.0, 0)
    UP = (0, 0, 1.0)
    DOWN = (0, 0, -1.0)

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    @property
    def vector(self):
        return Vector3(x=self.x, y=self.y, z=self.z)


class GazeboTopicName(object):
    """Gazebo Topic Names"""
    MODEL_STATES = '/gazebo/model_states'
    LINK_STATES = '/gazebo/link_states'
    VISUALS = '/gazebo/visuals'


class GazeboServiceName(object):
    """Gazebo Service Names"""
    GET_MODEL_STATE = '/gazebo/get_model_state'
    GET_MODEL_STATES = '/gazebo/get_model_states'
    GET_ALL_MODEL_STATES = '/gazebo/get_all_model_states'
    GET_LINK_STATE = '/gazebo/get_link_state'
    GET_LINK_STATES = '/gazebo/get_link_states'
    GET_ALL_LINK_STATES = '/gazebo/get_all_link_states'
    GET_MODEL_PROPERTIES = '/gazebo/get_model_properties'
    GET_LIGHT_NAMES = '/gazebo/get_light_names'
    GET_VISUAL_NAMES = '/gazebo/get_visual_names'
    GET_VISUAL = '/gazebo/get_visual'
    GET_VISUALS = '/gazebo/get_visuals'
    GET_ALL_VISUALS = '/gazebo/get_all_visuals'
    SET_MODEL_STATE = '/gazebo/set_model_state'
    SET_MODEL_STATES = '/gazebo/set_model_states'
    SET_LINK_STATE = '/gazebo/set_link_state'
    SET_LINK_STATES = '/gazebo/set_link_states'
    SET_LIGHT_PROPERTIES = '/gazebo/set_light_properties'
    SET_VISUAL_MATERIAL = '/gazebo/set_visual_material'
    SET_VISUAL_MATERIALS = '/gazebo/set_visual_materials'
    SET_VISUAL_TRANSPARENCY = '/gazebo/set_visual_transparency'
    SET_VISUAL_TRANSPARENCIES = '/gazebo/set_visual_transparencies'
    SET_VISUAL_VISIBLE = '/gazebo/set_visual_visible'
    SET_VISUAL_VISIBLES = '/gazebo/set_visual_visibles'
    PAUSE_PHYSICS = '/gazebo/pause_world_physics'
    UNPAUSE_PHYSICS = '/gazebo/unpause_world_physics'
    SPAWN_URDF_MODEL = '/gazebo/spawn_urdf_model'
    SPAWN_SDF_MODEL = '/gazebo/spawn_sdf_model'
    DELETE_MODEL = '/gazebo/delete_model'


class GeometryType(Enum):
    """Geometry Type"""
    BOX = 1
    CYLINDER = 2
    SPHERE = 3
    PLANE = 4
    IMAGE = 5
    HEIGHTMAP = 6
    MESH = 7
    TRIANGLE_FAN = 8
    LINE_STRIP = 9
    POLYLINE = 10
    EMPTY = 11
