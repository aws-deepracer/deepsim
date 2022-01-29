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
"""A class for set_visual_transparency tracker."""
import threading
from typing import Collection
from collections import OrderedDict

from deepsim.gazebo.constants import GazeboServiceName
from deepsim.sim_trackers.tracker import TrackerInterface
from deepsim.sim_trackers.tracker_manager import TrackerManager
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
import deepsim.sim_trackers.constants as consts
from deepsim.sim_trackers.trackers.get_visual_tracker import GetVisualTracker

from deepsim_msgs.srv import (
    SetVisualTransparencies, SetVisualTransparenciesRequest, SetVisualTransparenciesResponse,
    SetVisualTransparency, SetVisualTransparencyResponse
)
from rosgraph_msgs.msg import Clock


class SetVisualTransparencyTracker(TrackerInterface):
    """
    SetVisualTransparency Tracker class
    """
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'SetVisualTransparencyTracker':
        """
        Method for getting a reference to the SetVisualTransparency Tracker object

        Returns:
            SetVisualTransparencyTracker: SetVisualTransparencyTracker instance
        """
        with SetVisualTransparencyTracker._instance_lock:
            if SetVisualTransparencyTracker._instance is None:
                SetVisualTransparencyTracker()
            return SetVisualTransparencyTracker._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize SetVisualTransparency Tracker

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if SetVisualTransparencyTracker._instance is not None:
                raise RuntimeError("Attempting to construct multiple SetVisualTransparency Tracker")
            SetVisualTransparencyTracker._instance = self

        self._lock = threading.RLock()
        self._visual_name_map = OrderedDict()
        self._link_name_map = OrderedDict()
        self._transparency_map = OrderedDict()

        self._set_visual_transparencies = ServiceProxyWrapper(GazeboServiceName.SET_VISUAL_TRANSPARENCIES,
                                                              SetVisualTransparencies)

        TrackerManager.get_instance().add(tracker=self, priority=consts.TrackerPriority.LOW)

    def set_visual_transparency(self, link_name: str, visual_name: str,
                                transparency: float, blocking: bool = False) -> SetVisualTransparencyResponse:
        """
        Set transparency that will be updated in next update call
        * If blocking is True, it will be updated synchronously.

        Args:
            visual_name (str): name of visual
            link_name (str):  name of the link holding visual
            transparency (float): visual's transparency between 0.0 (opaque) and 1.0 (full transparent)
            blocking (bool): flag to block or not

        Returns:
            SetVisualTransparencyResponse: response msg
        """
        msg = SetVisualTransparencyResponse()
        key = (link_name, visual_name)
        with self._lock:
            if blocking:
                if key in self._visual_name_map:
                    del self._visual_name_map[key]
                    del self._link_name_map[key]
                    del self._transparency_map[key]
                req = SetVisualTransparenciesRequest()
                req.visual_names = [visual_name]
                req.link_names = [link_name]
                req.transparencies = [transparency]
                req.block = True
                res = self._set_visual_transparencies(req)
                msg.success = res.success and res.status[0]
                if msg.success:
                    GetVisualTracker.get_instance().set_transparency(link_name=link_name,
                                                                     visual_name=visual_name,
                                                                     transparency=transparency)
                msg.status_message = res.messages[0] if res.success else res.status_message
            else:
                self._visual_name_map[key] = visual_name
                self._link_name_map[key] = link_name
                self._transparency_map[key] = transparency
        return msg

    def set_visual_transparencies(self, link_names: Collection[str], visual_names: Collection[str],
                                  transparencies: Collection[float],
                                  blocking: bool = False) -> SetVisualTransparenciesResponse:
        """
        Set transparency that will be updated in next update call
        * If blocking is True, it will be updated synchronously.

        Args:
            link_names (Collection[str]):  names of the link holding visual
            visual_names (Collection[str]): names of visual
            transparencies (Collection[float]): visual's transparencies between 0.0 (opaque) and 1.0 (full transparent)
            blocking (bool): flag to block or not
        Returns:
            SetVisualTransparenciesResponse: response msg
        """
        msg = SetVisualTransparenciesResponse()
        msg.success = True

        with self._lock:
            if len(link_names) != len(visual_names) or len(link_names) != len(transparencies):
                err_msg = "link_names ({}), visual_names ({}), and transparencies ({}) ".format(len(link_names),
                                                                                                len(visual_names),
                                                                                                len(transparencies))
                err_msg += "must be equal size!"
                raise ValueError(err_msg)

            if blocking:
                for link_name, visual_name in zip(link_names, visual_names):
                    key = (link_name, visual_name)
                    if key in self._visual_name_map:
                        del self._visual_name_map[key]
                        del self._link_name_map[key]
                        del self._transparency_map[key]

                req = SetVisualTransparenciesRequest()
                req.visual_names = visual_names
                req.link_names = link_names
                req.transparencies = [transparency for transparency in transparencies]
                req.block = True
                msg = self._set_visual_transparencies(req)
                for status, link_name, visual_name, transparency in zip(msg.status, link_names, visual_names, transparencies):
                    if status:
                        GetVisualTracker.get_instance().set_transparency(link_name=link_name,
                                                                         visual_name=visual_name,
                                                                         transparency=transparency)
            else:
                for link_name, visual_name, transparency in zip(link_names, visual_names, transparencies):
                    key = (link_name, visual_name)
                    self._visual_name_map[key] = visual_name
                    self._link_name_map[key] = link_name
                    self._transparency_map[key] = transparency
                    msg.status.append(True)
                    msg.messages.append('')
        return msg

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update all transparencies tracking to gazebo

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        with self._lock:
            if self._visual_name_map.values():
                req = SetVisualTransparenciesRequest()

                req.visual_names = list(self._visual_name_map.values())
                req.link_names = list(self._link_name_map.values())
                req.transparencies = list(self._transparency_map.values())
                req.block = True
                # Request the call in async to operate the tracker in non-blocking manner.
                self._set_visual_transparencies(req)

            self._visual_name_map = OrderedDict()
            self._link_name_map = OrderedDict()
            self._transparency_map = OrderedDict()
