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
"""A class for get_visual tracker."""
from collections import OrderedDict
import threading
from typing import Collection, Dict, Tuple

from deepsim.exception import DeepSimException
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.sim_trackers.tracker import TrackerInterface
from deepsim.sim_trackers.tracker_manager import TrackerManager
from deepsim.math.vector3 import Vector3
from deepsim.math.pose import Pose
from deepsim.math.color import Color
from deepsim.math.material import Material
from deepsim.math.visual import Visual
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
import deepsim.sim_trackers.constants as consts

from deepsim_msgs.srv import (
    GetVisuals,
    GetAllVisuals,
    GetAllVisualsRequest
)
from rosgraph_msgs.msg import Clock


class GetVisualTracker(TrackerInterface):
    """
    GetVisual Tracker class
    """
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'GetVisualTracker':
        """
        Method for getting a reference to the GetVisual Tracker object

        Returns:
            GetVisualTracker: GetVisualTracker instance
        """
        with GetVisualTracker._instance_lock:
            if GetVisualTracker._instance is None:
                GetVisualTracker()
            return GetVisualTracker._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize GetVisual Tracker

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if GetVisualTracker._instance is not None:
                raise RuntimeError("Attempting to construct multiple GetVisual Tracker")
            GetVisualTracker._instance = self

        self._lock = threading.RLock()
        self._visual_map = {}

        self._get_visuals = ServiceProxyWrapper(GazeboServiceName.GET_VISUALS, GetVisuals)
        self._get_all_visuals = ServiceProxyWrapper(GazeboServiceName.GET_ALL_VISUALS, GetAllVisuals)

        TrackerManager.get_instance().add(tracker=self, priority=consts.TrackerPriority.HIGH)

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update all link states tracking to gazebo

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        visual_map = {}
        res = self._get_all_visuals(GetAllVisualsRequest())
        if res.success:
            for visual in res.visuals:
                key = (visual.link_name, visual.visual_name)
                visual_map[key] = Visual.from_ros(visual)
            with self._lock:
                self._visual_map = visual_map

    def get_visual(self, link_name: str, visual_name: str,
                   blocking: bool = False) -> Visual:
        """
        Return visual of given name of the link + visual name.

        Args:
            link_name (str): name of the link.
            visual_name (str): the visual name.
            blocking (bool): flag to block or not

        Returns:
            Visual: visual
        """
        key = (link_name, visual_name)
        with self._lock:
            if blocking or key not in self._visual_map:
                res = self._get_visuals([link_name], [visual_name])
                if res.success and res.status[0]:
                    visual = res.visuals[0]
                    return Visual(link_name=visual.link_name,
                                  visual_name=visual.visual_name,
                                  material=Material(ambient=Color.from_ros(visual.ambient),
                                                    diffuse=Color.from_ros(visual.diffuse),
                                                    specular=Color.from_ros(visual.specular),
                                                    emissive=Color.from_ros(visual.emissive)),
                                  transparency=visual.transparency,
                                  visible=visual.visible,
                                  geometry_type=visual.geometry_type,
                                  mesh_geom_filename=visual.mesh_geom_filename,
                                  mesh_geom_scale=Vector3.from_ros(visual.mesh_geom_scale),
                                  pose=Pose.from_ros(visual.pose))
                else:
                    err_message = res.messages[0] if res.messages else ''
                    raise DeepSimException("get_visuals failed: {} ({})".format(res.status_message,
                                                                                err_message))
            else:
                return self._visual_map[key].copy()

    def get_visuals(self, link_names: Collection[str], visual_names: Collection[str],
                    blocking: bool = False) -> Dict[Tuple[str, str], Visual]:
        """
        Return visual of given name of the link + visual name.

        Args:
            link_names (Collection[str]): names of the link.
            visual_names (Collection[str]): the visual names.
            blocking (bool): flag to block or not

        Returns:
            Dict[Tuple[str, str], Visual]: {(link_name, visual_name): visual}
        """
        if len(link_names) != len(visual_names):
            err_msg = "link_names ({}) and visual_names ({}) must be equal size!".format(len(link_names),
                                                                                         len(visual_names))
            raise ValueError(err_msg)

        visuals = OrderedDict()

        query_link_names = []
        query_visual_names = []
        with self._lock:
            for link_name, visual_name in zip(link_names, visual_names):
                key = (link_name, visual_name)
                if blocking or key not in self._visual_map:
                    query_link_names.append(link_name)
                    query_visual_names.append(visual_name)
                    visuals[key] = Visual()
                else:
                    visuals[key] = self._visual_map[key].copy()

        if len(query_link_names) > 0 and len(query_visual_names) > 0:
            res = self._get_visuals(query_link_names, query_visual_names)
            if res.success:
                for idx, visual in enumerate(res.visuals):
                    key = (query_link_names[idx], query_visual_names[idx])
                    if res.status[idx]:
                        visuals[key] = Visual(link_name=visual.link_name,
                                              visual_name=visual.visual_name,
                                              material=Material(ambient=Color.from_ros(visual.ambient),
                                                                diffuse=Color.from_ros(visual.diffuse),
                                                                specular=Color.from_ros(visual.specular),
                                                                emissive=Color.from_ros(visual.emissive)),
                                              transparency=visual.transparency,
                                              visible=visual.visible,
                                              geometry_type=visual.geometry_type,
                                              mesh_geom_filename=visual.mesh_geom_filename,
                                              mesh_geom_scale=Vector3.from_ros(visual.mesh_geom_scale),
                                              pose=Pose.from_ros(visual.pose))
                    else:
                        visuals[key] = None
            else:
                raise DeepSimException("get_visuals failed: {}".format(res.status_message))
        return visuals

    def set_transparency(self, link_name: str, visual_name: str, transparency: float) -> None:
        """
        Set given transparency to the cache.

        Args:
            link_name (str): link name
            visual_name (str): visual name
            transparency (float): transparency to cache.
        """
        with self._lock:
            key = (link_name, visual_name)
            if key in self._visual_map:
                self._visual_map[key].transparency = transparency

    def set_material(self, link_name: str, visual_name: str, material: Material) -> None:
        """
        Set given material to the cache.

        Args:
            link_name (str): link name
            visual_name (str): visual name
            material (Material): material to cache.
        """
        with self._lock:
            key = (link_name, visual_name)
            if key in self._visual_map:
                self._visual_map[key].material = material

    def set_visible(self, link_name: str, visual_name: str, visible: bool) -> None:
        """
        Set given visible to the cache.

        Args:
            link_name (str): link name
            visual_name (str): visual name
            visible (bool): visible to cache.
        """
        with self._lock:
            key = (link_name, visual_name)
            if key in self._visual_map:
                self._visual_map[key].visible = visible
