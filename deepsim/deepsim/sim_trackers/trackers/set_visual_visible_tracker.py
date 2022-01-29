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
"""A class for set_visual_visible tracker."""
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
    SetVisualVisibles, SetVisualVisiblesRequest, SetVisualVisiblesResponse,
    SetVisualVisible, SetVisualVisibleResponse
)
from rosgraph_msgs.msg import Clock


class SetVisualVisibleTracker(TrackerInterface):
    """
    SetVisualVisible Tracker class
    """
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'SetVisualVisibleTracker':
        """
        Method for getting a reference to the SetVisualVisible Tracker object

        Returns:
            SetVisualVisibleTracker: SetVisualVisibleTracker instance
        """
        with SetVisualVisibleTracker._instance_lock:
            if SetVisualVisibleTracker._instance is None:
                SetVisualVisibleTracker()
            return SetVisualVisibleTracker._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize SetVisualVisible Tracker

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if SetVisualVisibleTracker._instance is not None:
                raise RuntimeError("Attempting to construct multiple SetVisualVisible Tracker")
            SetVisualVisibleTracker._instance = self

        self._lock = threading.RLock()
        self._visual_name_map = OrderedDict()
        self._link_name_map = OrderedDict()
        self._visible_map = OrderedDict()

        self._set_visual_visibles = ServiceProxyWrapper(GazeboServiceName.SET_VISUAL_VISIBLES,
                                                        SetVisualVisibles)

        TrackerManager.get_instance().add(tracker=self, priority=consts.TrackerPriority.LOW)

    def set_visual_visible(self, link_name: str, visual_name: str,
                           visible: bool, blocking: bool = False) -> SetVisualVisibleResponse:
        """
        Set visible that will be updated in next update call
        * If blocking is True, it will be updated synchronously.

        Args:
            visual_name (str): name of visual
            link_name (str):  name of the link holding visual
            visible (bool): visual's visibility
            blocking (bool): flag to block or not

        Returns:
            SetVisualVisibleResponse: response msg
        """
        msg = SetVisualVisibleResponse()
        key = (link_name, visual_name)
        with self._lock:
            if blocking:
                if key in self._visual_name_map:
                    del self._visual_name_map[key]
                    del self._link_name_map[key]
                    del self._visible_map[key]
                req = SetVisualVisiblesRequest()
                req.visual_names = [visual_name]
                req.link_names = [link_name]
                req.visibles = [visible]
                req.block = True
                res = self._set_visual_visibles(req)
                msg.success = res.success and res.status[0]
                if msg.success:
                    GetVisualTracker.get_instance().set_visible(link_name=link_name,
                                                                visual_name=visual_name,
                                                                visible=visible)
                msg.status_message = res.messages[0] if res.success else res.status_message
            else:
                self._visual_name_map[key] = visual_name
                self._link_name_map[key] = link_name
                self._visible_map[key] = visible
        return msg

    def set_visual_visibles(self, link_names: Collection[str], visual_names: Collection[str],
                            visibles: Collection[bool],
                            blocking: bool = False) -> SetVisualVisiblesResponse:
        """
        Set visible that will be updated in next update call
        * If blocking is True, it will be updated synchronously.

        Args:
            link_names (Collection[str]):  names of the link holding visual
            visual_names (Collection[str]): names of visual
            visibles (Collection[bool]): visual's visibility
            blocking (bool): flag to block or not
        Returns:
            SetVisualVisiblesResponse: response msg
        """
        msg = SetVisualVisiblesResponse()
        msg.success = True

        with self._lock:
            if len(link_names) != len(visual_names) or len(link_names) != len(visibles):
                err_msg = "link_names ({}), visual_names ({}), and visibles ({}) ".format(len(link_names),
                                                                                          len(visual_names),
                                                                                          len(visibles))
                err_msg += "must be equal size!"
                raise ValueError(err_msg)

            if blocking:
                for link_name, visual_name in zip(link_names, visual_names):
                    key = (link_name, visual_name)
                    if key in self._visual_name_map:
                        del self._visual_name_map[key]
                        del self._link_name_map[key]
                        del self._visible_map[key]

                req = SetVisualVisiblesRequest()
                req.visual_names = visual_names
                req.link_names = link_names
                req.visibles = [visible for visible in visibles]
                req.block = True
                msg = self._set_visual_visibles(req)
                for status, link_name, visual_name, visible in zip(msg.status, link_names, visual_names, visibles):
                    if status:
                        GetVisualTracker.get_instance().set_visible(link_name=link_name,
                                                                    visual_name=visual_name,
                                                                    visible=visible)
            else:
                for link_name, visual_name, visible in zip(link_names, visual_names, visibles):
                    key = (link_name, visual_name)
                    self._visual_name_map[key] = visual_name
                    self._link_name_map[key] = link_name
                    self._visible_map[key] = visible
                    msg.status.append(True)
                    msg.messages.append('')
        return msg

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update all visible tracking to gazebo

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        with self._lock:
            if self._visual_name_map.values():
                req = SetVisualVisiblesRequest()

                req.visual_names = list(self._visual_name_map.values())
                req.link_names = list(self._link_name_map.values())
                req.visibles = list(self._visible_map.values())
                req.block = True
                # Request the call in async to operate the tracker in non-blocking manner.
                self._set_visual_visibles(req)

            self._visual_name_map = OrderedDict()
            self._link_name_map = OrderedDict()
            self._visible_map = OrderedDict()
