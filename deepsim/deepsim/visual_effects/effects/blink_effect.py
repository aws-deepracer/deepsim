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
"""A class for blink effect."""
from deepsim.core.math import lerp
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
from deepsim.visual_effects.abs_effect import AbstractEffect
from deepsim.sim_trackers.trackers.set_visual_transparency_tracker import SetVisualTransparencyTracker

from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from deepsim_msgs.srv import (
    GetVisualNames, GetVisualNamesRequest,
    GetVisuals, GetVisualsRequest
)
from rosgraph_msgs.msg import Clock


class BlinkEffect(AbstractEffect):
    """
    Blink Visual Effect
    """
    def __init__(self, model_name: str,
                 min_alpha: float = 0.3, max_alpha: float = 1.0,
                 interval: float = 1.0, duration: float = 2.0) -> None:
        """
        Constructor

        Args:
            model_name (str): name of the model
            min_alpha (float): minimum alpha for blink
            max_alpha (float): maximum alpha for blink
            interval (float): interval in second for one cycle of blink
            duration (float): effect duration in second
        """
        super(BlinkEffect, self).__init__()
        self._model_name = model_name
        self._min_alpha = min_alpha
        self._max_alpha = max_alpha
        self._interval = interval
        self._duration = duration

        self._source_alpha = self._min_alpha
        self._target_alpha = self._max_alpha
        self._current_interval = 0.0
        self._current_duration = 0.0

    @property
    def model_name(self) -> str:
        """
        Returns the name of the model.

        Returns:
            str: the name of the model.
        """
        return self._model_name

    @property
    def min_alpha(self) -> float:
        """
        Returns the minimum alpha [0.0 1.0].

        Returns:
            float: the minimum alpha.
        """
        return self._min_alpha

    @property
    def max_alpha(self) -> float:
        """
        Returns the maximum alpha [0.0 1.0].

        Returns:
            float: the maximum alpha.
        """
        return self._max_alpha

    @property
    def interval(self) -> float:
        """
        Returns the interval in second for one cycle of blink.

        Returns:
            float: the interval in second for one cycle of blink.
        """
        return self._interval

    @property
    def duration(self) -> float:
        """
        Returns the effect duration in second.

        Returns:
            float: the effect duration in second.
        """
        return self._duration

    @property
    def current_interval(self) -> float:
        """
        Returns the seconds passed within current interval.

        Returns:
            float: the seconds passed within current interval.
        """
        return self._current_interval

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
        self._source_alpha = self._min_alpha
        self._target_alpha = self._max_alpha
        self._current_interval = 0.0
        self._current_duration = 0.0

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
        During attach, reset the effect.
        """
        self.reset()

    def on_detach_effect(self) -> None:
        """
        After detach, reset transparencies to original.
        """
        if self.is_initialized:
            for visual in self._orig_visuals.visuals:
                SetVisualTransparencyTracker.get_instance().set_visual_transparency(visual.link_name,
                                                                                    visual.visual_name,
                                                                                    visual.transparency)

    def on_update_effect(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update blink effect

        Args:
            delta_time (float): time diff from last call
            sim_time (Clock): simulation time
        """
        if self._current_duration < self._duration:
            cur_alpha = lerp(self._source_alpha, self._target_alpha, self._current_interval / self._interval)
            transparency = 1.0 - cur_alpha
            for visual in self._orig_visuals.visuals:
                SetVisualTransparencyTracker.get_instance().set_visual_transparency(visual.link_name,
                                                                                    visual.visual_name,
                                                                                    transparency)

            self._current_interval += delta_time
            if self._current_interval >= self._interval:
                temp = self._source_alpha
                self._source_alpha = self._target_alpha
                self._target_alpha = temp
                self._current_interval = 0.0
            self._current_duration += delta_time
        else:
            self.detach()
