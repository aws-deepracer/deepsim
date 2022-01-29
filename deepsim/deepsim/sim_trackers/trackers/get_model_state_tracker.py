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
"""A class for get_model_state tracker."""
from collections import OrderedDict
import threading
from typing import Optional, Collection, Dict, Tuple

from deepsim.exception import DeepSimException
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.sim_trackers.tracker import TrackerInterface
from deepsim.sim_trackers.tracker_manager import TrackerManager
from deepsim.math.twist import Twist
from deepsim.math.pose import Pose
from deepsim.math.model_state import ModelState
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
import deepsim.sim_trackers.constants as consts

from deepsim_msgs.srv import (
    GetModelStates,
    GetAllModelStates,
    GetAllModelStatesRequest
)
from rosgraph_msgs.msg import Clock


class GetModelStateTracker(TrackerInterface):
    """
    GetModelState Tracker class
    """
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'GetModelStateTracker':
        """
        Method for getting a reference to the GetModelState Tracker object

        Returns:
            GetModelStateTracker: GetModelStateTracker instance
        """
        with GetModelStateTracker._instance_lock:
            if GetModelStateTracker._instance is None:
                GetModelStateTracker()
            return GetModelStateTracker._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize GetModelState Tracker

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if GetModelStateTracker._instance is not None:
                raise RuntimeError("Attempting to construct multiple GetModelState Tracker")
            GetModelStateTracker._instance = self

        self._lock = threading.RLock()
        self._model_map = {}

        self._get_model_states = ServiceProxyWrapper(GazeboServiceName.GET_MODEL_STATES, GetModelStates)
        self._get_all_model_states = ServiceProxyWrapper(GazeboServiceName.GET_ALL_MODEL_STATES, GetAllModelStates)
        TrackerManager.get_instance().add(tracker=self, priority=consts.TrackerPriority.HIGH)

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update all model states tracking to gazebo

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        model_map = {}
        res = self._get_all_model_states(GetAllModelStatesRequest())
        if res.success:
            for model_state in res.model_states:
                model_map[model_state.model_name] = ModelState.from_ros(model_state)
            with self._lock:
                self._model_map = model_map

    def get_model_state(self, name: str, reference_frame: Optional[str] = None,
                        blocking: bool = False) -> ModelState:
        """
        Return model state of given name of the model.

        Args:
            name (str): name of the model.
            reference_frame (Optional[str]): the reference frame
            blocking (bool): flag to block or not

        Returns:
            ModelState: model state
        """
        with self._lock:
            if blocking or name not in self._model_map or reference_frame:
                # if name doesn't exist in the map or if there is reference frame specified
                # then manually retrieve model_state
                reference_frame = reference_frame if reference_frame else ''
                res = self._get_model_states([name], [reference_frame])
                if res.success and res.status[0]:
                    model_state = res.model_states[0]
                    return ModelState(model_state.model_name,
                                      pose=Pose.from_ros(model_state.pose),
                                      twist=Twist.from_ros(model_state.twist),
                                      reference_frame=model_state.reference_frame)
                else:
                    err_message = res.messages[0] if res.messages else ''
                    raise DeepSimException("get_model_state failed: {} ({})".format(res.status_message,
                                                                                    err_message))
            else:
                return self._model_map[name].copy()

    def get_model_states(self, names: Collection[str], reference_frames: Optional[Collection[str]] = None,
                         blocking: bool = False) -> Dict[Tuple[str, str], ModelState]:
        """
        Return model state of given name of the model.
        * This method will ignore model_state that is failed to retrieve.

        Args:
            names (Collection[str]): name of the model.
            reference_frames (Optional[Collection[str]]): the reference frames
            blocking (bool): flag to block or not

        Returns:
            Dict[Tuple[str, str], ModelState]: {(name, reference_frame): model_state}
        """
        models = OrderedDict()
        if reference_frames is None:
            reference_frames = ['' for _ in names]

        if len(names) != len(reference_frames):
            err_msg = "names ({}) and reference_frames ({}) must be equal size!".format(len(names),
                                                                                        len(reference_frames))
            raise ValueError(err_msg)

        query_names = []
        query_reference_frames = []
        with self._lock:
            for name, reference_frame in zip(names, reference_frames):
                key = (name, reference_frame)
                if blocking or name not in self._model_map or reference_frame:
                    query_names.append(name)
                    query_reference_frames.append(reference_frame)
                    models[key] = ModelState()
                else:
                    models[key] = self._model_map[name].copy()

        if len(query_names) > 0 and len(query_reference_frames) > 0:
            res = self._get_model_states(query_names, query_reference_frames)
            if res.success:
                for idx, model_state in enumerate(res.model_states):
                    key = (query_names[idx], query_reference_frames[idx])
                    if res.status[idx]:
                        models[key] = ModelState(model_state.model_name,
                                                 pose=Pose.from_ros(model_state.pose),
                                                 twist=Twist.from_ros(model_state.twist),
                                                 reference_frame=model_state.reference_frame)
                    else:
                        models[key] = None
            else:
                raise DeepSimException("get_model_state failed: {}".format(res.status_message))
        return models

    def set_model_state(self, model_state: ModelState) -> None:
        """
        Set given ModelState to cache.

        Args:
            model_state (ModelState): model state to cache.
        """
        with self._lock:
            self._model_map[model_state.model_name] = model_state.copy()
