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
"""A class for set_link_state tracker."""
import threading
from typing import Iterable

from deepsim.gazebo.constants import GazeboServiceName
from deepsim.sim_trackers.tracker import TrackerInterface
from deepsim.sim_trackers.tracker_manager import TrackerManager
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
import deepsim.sim_trackers.constants as consts
from deepsim.core.link_state import LinkState
from deepsim.sim_trackers.trackers.get_link_state_tracker import GetLinkStateTracker

from deepsim_msgs.srv import SetLinkStates, SetLinkStatesResponse
from gazebo_msgs.srv import SetLinkState, SetLinkStateResponse
from rosgraph_msgs.msg import Clock


class SetLinkStateTracker(TrackerInterface):
    """
    SetLinkState Tracker class
    """
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'SetLinkStateTracker':
        """
        Method for getting a reference to the SetLinkState Tracker object

        Returns:
            SetLinkStateTracker: SetLinkStateTracker instance
        """
        with SetLinkStateTracker._instance_lock:
            if SetLinkStateTracker._instance is None:
                SetLinkStateTracker()
            return SetLinkStateTracker._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize SetLinkState Tracker

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if SetLinkStateTracker._instance is not None:
                raise RuntimeError("Attempting to construct multiple SetLinkState Tracker")
            SetLinkStateTracker._instance = self

        self._lock = threading.RLock()
        self._link_state_map = {}

        self._set_link_states = ServiceProxyWrapper(GazeboServiceName.SET_LINK_STATES, SetLinkStates)

        TrackerManager.get_instance().add(tracker=self, priority=consts.TrackerPriority.LOW)

    def set_link_state(self, link_state: LinkState, blocking: bool = False) -> SetLinkStateResponse:
        """
        Set LinkState that will be updated in next update call

        Args:
            link_state (LinkState): the link state to update
            blocking (bool): flag to block or not
        Returns:
            SetLinkStateResponse: response msg
        """
        msg = SetLinkStateResponse()
        msg.success = True
        ros_link_state = link_state.to_ros()
        with self._lock:
            if blocking:
                if ros_link_state.link_name in self._link_state_map:
                    del self._link_state_map[ros_link_state.link_name]
                res = self._set_link_states([ros_link_state])
                msg.success = res.success and res.status[0]
                if msg.success:
                    GetLinkStateTracker.get_instance().set_link_state(link_state=link_state)
                msg.status_message = res.messages[0] if res.success else res.status_message
            else:
                self._link_state_map[ros_link_state.link_name] = ros_link_state
        return msg

    def set_link_states(self, link_states: Iterable[LinkState], blocking: bool = False) -> SetLinkStatesResponse:
        """
        Set LinkState that will be updated in next update call
        * If blocking is True, it will be updated synchronously.

        Args:
            link_states (Iterable[LinkState]): the link states to update
            blocking (bool): flag to block or not
        Returns:
            SetLinkStateResponse: response msg
        """
        msg = SetLinkStatesResponse()
        msg.success = True

        with self._lock:
            if blocking:
                ros_link_states = []
                for link_state in link_states:
                    ros_link_state = link_state.to_ros()
                    ros_link_states.append(ros_link_state)
                    if ros_link_state.link_name in self._link_state_map:
                        del self._link_state_map[ros_link_state.link_name]
                msg = self._set_link_states(ros_link_states)
                for status, link_state in zip(msg.status, link_states):
                    if status:
                        GetLinkStateTracker.get_instance().set_link_state(link_state=link_state)
            else:
                for link_state in link_states:
                    ros_link_state = link_state.to_ros()
                    self._link_state_map[ros_link_state.link_name] = ros_link_state
                    msg.status.append(True)
                    msg.messages.append('')
        return msg

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update all link states tracking to gazebo

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        with self._lock:
            if self._link_state_map.values():
                # Request the call in async to operate the tracker in non-blocking manner.
                self._set_link_states(list(self._link_state_map.values()))
            self._link_state_map = {}
