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
"""A class for set_model_state tracker."""
import threading
from typing import Iterable

from deepsim.gazebo.constants import GazeboServiceName
from deepsim.sim_trackers.tracker import TrackerInterface
from deepsim.sim_trackers.tracker_manager import TrackerManager
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
import deepsim.sim_trackers.constants as consts
from deepsim.math.model_state import ModelState
from deepsim.sim_trackers.trackers.get_model_state_tracker import GetModelStateTracker

from deepsim_msgs.srv import SetModelStates, SetModelStatesResponse
from gazebo_msgs.srv import SetModelState, SetModelStateResponse
from rosgraph_msgs.msg import Clock


class SetModelStateTracker(TrackerInterface):
    """
    SetModelState Tracker class
    """
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'SetModelStateTracker':
        """
        Method for getting a reference to the SetModelState Tracker object

        Returns:
            SetModelStateTracker: SetModelStateTracker instance
        """
        with SetModelStateTracker._instance_lock:
            if SetModelStateTracker._instance is None:
                SetModelStateTracker()
            return SetModelStateTracker._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize SetModelState Tracker

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if SetModelStateTracker._instance is not None:
                raise RuntimeError("Attempting to construct multiple SetModelState Tracker")
            SetModelStateTracker._instance = self

        self._lock = threading.RLock()
        self._model_state_map = {}

        self._set_model_states = ServiceProxyWrapper(GazeboServiceName.SET_MODEL_STATES, SetModelStates)

        TrackerManager.get_instance().add(tracker=self, priority=consts.TrackerPriority.LOW)

    def set_model_state(self, model_state: ModelState, blocking: bool = False) -> SetModelStateResponse:
        """
        Set ModelState that will be updated in next update call

        Args:
            model_state (ModelState): the model state to update
            blocking (bool): flag to block or not
        Returns:
            SetModelStateResponse: response msg
        """
        msg = SetModelStateResponse()
        msg.success = True
        ros_model_state = model_state.to_ros()
        with self._lock:
            if blocking:
                if ros_model_state.model_name in self._model_state_map:
                    del self._model_state_map[ros_model_state.model_name]
                res = self._set_model_states([ros_model_state])
                msg.success = res.success and res.status[0]
                if msg.success:
                    GetModelStateTracker.get_instance().set_model_state(model_state=model_state)
                msg.status_message = res.messages[0] if res.success else res.status_message
            else:
                self._model_state_map[ros_model_state.model_name] = ros_model_state
        return msg

    def set_model_states(self, model_states: Iterable[ModelState], blocking: bool = False) -> SetModelStatesResponse:
        """
        Set ModelState that will be updated in next update call
        * If blocking is True, it will be updated synchronously.

        Args:
            model_states (Iterable[ModelState]): the model states to update
            blocking (bool): flag to block or not
        Returns:
            SetModelStateResponse: response msg
        """
        msg = SetModelStatesResponse()
        msg.success = True

        with self._lock:
            if blocking:
                ros_model_states = []
                for model_state in model_states:
                    ros_model_state = model_state.to_ros()
                    ros_model_states.append(ros_model_state)
                    if ros_model_state.model_name in self._model_state_map:
                        del self._model_state_map[ros_model_state.model_name]
                msg = self._set_model_states(ros_model_states)
                for status, model_state in zip(msg.status, model_states):
                    if status:
                        GetModelStateTracker.get_instance().set_model_state(model_state=model_state)
            else:
                for model_state in model_states:
                    ros_model_state = model_state.to_ros()
                    self._model_state_map[ros_model_state.model_name] = ros_model_state
                    msg.status.append(True)
                    msg.messages.append('')
        return msg

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update all model states tracking to gazebo

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        with self._lock:
            if self._model_state_map.values():
                # Request the call in async to operate the tracker in non-blocking manner.
                self._set_model_states(list(self._model_state_map.values()))
            self._model_state_map = {}
