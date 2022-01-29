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
"""A class for set_visual_material tracker."""
import threading
from typing import Collection
from collections import OrderedDict

from deepsim.gazebo.constants import GazeboServiceName
from deepsim.sim_trackers.tracker import TrackerInterface
from deepsim.sim_trackers.tracker_manager import TrackerManager
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
import deepsim.sim_trackers.constants as consts
from deepsim.math.material import Material
from deepsim.sim_trackers.trackers.get_visual_tracker import GetVisualTracker

from deepsim_msgs.srv import (
    SetVisualMaterials, SetVisualMaterialsRequest, SetVisualMaterialsResponse,
    SetVisualMaterial, SetVisualMaterialResponse
)
from rosgraph_msgs.msg import Clock


class SetVisualMaterialTracker(TrackerInterface):
    """
    SetVisualMaterialTracker Tracker class
    """
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'SetVisualMaterialTracker':
        """
        Method for getting a reference to the SetVisualMaterial Tracker object

        Returns:
            SetVisualMaterialTracker: SetVisualMaterialTracker instance
        """
        with SetVisualMaterialTracker._instance_lock:
            if SetVisualMaterialTracker._instance is None:
                SetVisualMaterialTracker()
            return SetVisualMaterialTracker._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize SetVisualMaterial Tracker

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if SetVisualMaterialTracker._instance is not None:
                raise RuntimeError("Attempting to construct multiple SetVisualMaterial Tracker")
            SetVisualMaterialTracker._instance = self

        self._lock = threading.RLock()
        self._visual_name_map = OrderedDict()
        self._link_name_map = OrderedDict()
        self._ambient_map = OrderedDict()
        self._diffuse_map = OrderedDict()
        self._specular_map = OrderedDict()
        self._emissive_map = OrderedDict()

        self._set_visual_materials = ServiceProxyWrapper(GazeboServiceName.SET_VISUAL_MATERIALS, SetVisualMaterials)

        TrackerManager.get_instance().add(tracker=self, priority=consts.TrackerPriority.LOW)

    def set_visual_material(self, link_name: str, visual_name: str,
                            material: Material,
                            blocking: bool = False) -> SetVisualMaterialResponse:
        """
        Set Material that will be updated in next update call
        * If blocking is True, it will be updated synchronously.

        Args:
            link_name (str):  name of the link holding visual
            visual_name (str): name of visual
            material (Material): color material
            blocking (bool): flag to block or not
        Returns:
            SetVisualMaterialResponse: response msg
        """
        msg = SetVisualMaterialResponse()
        msg.success = True
        key = (link_name, visual_name)
        with self._lock:
            if blocking:
                if key in self._visual_name_map:
                    del self._visual_name_map[key]
                    del self._link_name_map[key]
                    del self._ambient_map[key]
                    del self._diffuse_map[key]
                    del self._specular_map[key]
                    del self._emissive_map[key]

                req = SetVisualMaterialsRequest()
                req.visual_names = [visual_name]
                req.link_names = [link_name]
                req.ambients = [material.ambient.to_ros()]
                req.diffuses = [material.diffuse.to_ros()]
                req.speculars = [material.specular.to_ros()]
                req.emissives = [material.emissive.to_ros()]
                req.block = True
                res = self._set_visual_materials(req)
                msg.success = res.success and res.status[0]
                if msg.success:
                    GetVisualTracker.get_instance().set_material(link_name=link_name,
                                                                 visual_name=visual_name,
                                                                 material=material)
                msg.status_message = res.messages[0] if res.success else res.status_message
            else:
                self._visual_name_map[key] = visual_name
                self._link_name_map[key] = link_name
                self._ambient_map[key] = material.ambient.to_ros()
                self._diffuse_map[key] = material.diffuse.to_ros()
                self._specular_map[key] = material.specular.to_ros()
                self._emissive_map[key] = material.emissive.to_ros()
        return msg

    def set_visual_materials(self, link_names: Collection[str], visual_names: Collection[str],
                             materials: Collection[Material],
                             blocking: bool = False) -> SetVisualMaterialsResponse:
        """
        Set Materials that will be updated in next update call
        * If blocking is True, it will be updated synchronously.

        Args:
            link_names (Collection[str]):  names of the link holding visual
            visual_names (Collection[str]): names of visual
            materials (Collection[Material]): color materials
            blocking (bool): flag to block or not
        Returns:
            SetVisualMaterialsResponse: response msg
        """
        msg = SetVisualMaterialsResponse()
        msg.success = True

        with self._lock:
            if len(link_names) != len(visual_names) or len(link_names) != len(materials):
                err_msg = "link_names ({}), visual_names ({}), and materials ({}) ".format(len(link_names),
                                                                                           len(visual_names),
                                                                                           len(materials))
                err_msg += "must be equal size!"
                raise ValueError(err_msg)

            if blocking:
                for link_name, visual_name in zip(link_names, visual_names):
                    key = (link_name, visual_name)
                    if key in self._visual_name_map:
                        del self._visual_name_map[key]
                        del self._link_name_map[key]
                        del self._ambient_map[key]
                        del self._diffuse_map[key]
                        del self._specular_map[key]
                        del self._emissive_map[key]

                req = SetVisualMaterialsRequest()
                req.visual_names = visual_names
                req.link_names = link_names
                req.ambients = []
                req.diffuses = []
                req.speculars = []
                req.emissives = []
                for material in materials:
                    req.ambients.append(material.ambient.to_ros())
                    req.diffuses.append(material.diffuse.to_ros())
                    req.speculars.append(material.specular.to_ros())
                    req.emissives.append(material.emissive.to_ros())
                req.block = True
                msg = self._set_visual_materials(req)
                for status, link_name, visual_name, material in zip(msg.status, link_names, visual_names, materials):
                    if status:
                        GetVisualTracker.get_instance().set_material(link_name=link_name,
                                                                     visual_name=visual_name,
                                                                     material=material)
            else:
                for link_name, visual_name, material in zip(link_names, visual_names, materials):
                    key = (link_name, visual_name)
                    self._visual_name_map[key] = visual_name
                    self._link_name_map[key] = link_name
                    self._ambient_map[key] = material.ambient.to_ros()
                    self._diffuse_map[key] = material.diffuse.to_ros()
                    self._specular_map[key] = material.specular.to_ros()
                    self._emissive_map[key] = material.emissive.to_ros()
                    msg.status.append(True)
                    msg.messages.append('')
        return msg

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update all color materials tracking to gazebo

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        with self._lock:
            if self._visual_name_map.values():
                req = SetVisualMaterialsRequest()

                req.visual_names = list(self._visual_name_map.values())
                req.link_names = list(self._link_name_map.values())
                req.ambients = list(self._ambient_map.values())
                req.diffuses = list(self._diffuse_map.values())
                req.speculars = list(self._specular_map.values())
                req.emissives = list(self._emissive_map.values())
                req.block = True
                # Request the call in async to operate the tracker in non-blocking manner.
                self._set_visual_materials(req)

            self._visual_name_map = OrderedDict()
            self._link_name_map = OrderedDict()
            self._ambient_map = OrderedDict()
            self._diffuse_map = OrderedDict()
            self._specular_map = OrderedDict()
            self._emissive_map = OrderedDict()
