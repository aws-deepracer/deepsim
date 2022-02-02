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
"""A class for invisible effect."""
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
from deepsim.visual_effects.abs_effect import AbstractEffect
from deepsim.sim_trackers.trackers.set_visual_visible_tracker import SetVisualVisibleTracker

import rospy
from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from deepsim_msgs.srv import (
    GetVisualNames, GetVisualNamesRequest,
    GetVisuals, GetVisualsRequest
)


class InvisibleEffect(AbstractEffect):
    """Invisible Visual Effect"""
    def __init__(self, model_name: str, duration: float = 1.0) -> None:
        """
        Constructor

        Args:
            model_name (str): name of the model
            duration (float): effect duration in second
        """
        super(InvisibleEffect, self).__init__()
        self._model_name = model_name
        self._duration = duration
        self._current_duration = 0.0
        self._first_update_call = True

    @property
    def model_name(self) -> str:
        """
        Returns the name of the model.

        Returns:
            str: the name of the model.
        """
        return self._model_name

    @property
    def duration(self) -> float:
        """
        Returns the effect duration in second.

        Returns:
            float: the effect duration in second.
        """
        return self._duration

    @property
    def current_duration(self) -> float:
        """
        Returns the seconds passed from start of effect duration.

        Returns:
            float: the seconds passed from start of effect duration.
        """
        return self._current_duration

    def reset(self) -> None:
        """
        Reset internal variables
        """
        self._current_duration = 0.0
        self._first_update_call = True

    def _lazy_init(self) -> None:
        """
        Lazy-initialize effect
        """
        # ROS Services Setup
        get_model_prop = ServiceProxyWrapper(GazeboServiceName.GET_MODEL_PROPERTIES, GetModelProperties)
        get_visual_names = ServiceProxyWrapper(GazeboServiceName.GET_VISUAL_NAMES, GetVisualNames)
        get_visuals = ServiceProxyWrapper(GazeboServiceName.GET_VISUALS, GetVisuals)

        # Get all model's link names
        body_names = get_model_prop(GetModelPropertiesRequest(model_name=self._model_name)).body_names
        link_names = ["%s::%s" % (self._model_name, b) for b in body_names]

        res = get_visual_names(GetVisualNamesRequest(link_names=link_names))
        get_visuals_req = GetVisualsRequest(link_names=res.link_names,
                                            visual_names=res.visual_names)
        self._orig_visuals = get_visuals(get_visuals_req)

    def on_attach_effect(self) -> None:
        """
        During attach, reset internal variables.
        """
        self.reset()

    def on_detach_effect(self) -> None:
        """
        After detach, reset to original.
        """
        if self.is_initialized:
            for visual in self._orig_visuals.visuals:
                SetVisualVisibleTracker.get_instance().set_visual_visible(link_name=visual.link_name,
                                                                          visual_name=visual.visual_name,
                                                                          visible=visual.visible)

    def on_update_effect(self, delta_time, sim_time) -> None:
        """
        Update blink effect

        Args:
            delta_time (float): time diff from last call
            sim_time (Clock): simulation time
        """
        if self._current_duration < self._duration:
            if self._first_update_call:
                self._first_update_call = False

                for visual in self._orig_visuals.visuals:
                    SetVisualVisibleTracker.get_instance().set_visual_visible(link_name=visual.link_name,
                                                                              visual_name=visual.visual_name,
                                                                              visible=False)

            self._current_duration += delta_time
        else:
            self.detach()
