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
"""A class for model visual randomizer."""
import numpy as np
from typing import Union, Optional

from deepsim.domain_randomizations.abs_randomizer import AbstractRandomizer
from deepsim.domain_randomizations.constants import (
    ModelRandomizerType,
    ColorAttr,
    RANGE_MIN, RANGE_MAX
)
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.sim_trackers.trackers.set_visual_material_tracker import SetVisualMaterialTracker
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
from deepsim.core.color import Color
from deepsim.core.material import Material

from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from deepsim_msgs.srv import GetVisualNames, GetVisualNamesRequest


class ModelVisualRandomizer(AbstractRandomizer):
    """Model Visual Randomizer class"""
    def __init__(self, model_name: str, model_randomizer_type: ModelRandomizerType, num_selection: int = -1,
                 link_name_filter: Optional[Union[set, list]] = None,
                 visual_name_filter: Optional[Union[set, list]] = None,
                 color_range: Optional[dict] = None):
        """
        Constructor
        - Bit of explanation regarding model_randomizer_type:
            - There are 3 possible types (MODEL, LINK, VISUAL) due to the level of randomization.
              The reason is that a model's visual is represented in three level and their relationship as below:
              - 1 Model to N Links and 1 Link to M Visuals.
            - The one case that LINK may contain multiple VISUALs is that gazebo merges the links
              that is connected by fixed joint for the sake of physics performance. Even the links are merged
              together, it still needs to keep the visuals separately to display correctly with its own materials.
              Thus, single merged link can contain multiple visuals from the links before the merge.

        Args:
            model_name (str): name of the model
            model_randomizer_type (ModelRandomizerType): Model Randomizer Type
            num_selection (int): Number of visuals or link to select on each randomize. (-1 means all)
                                 (Only used for ModelRandomizerType.LINK or ModelRandomizerType.VISUAL)
            link_name_filter (set or list): If link_name_filter are provided,
                                            randomization will only apply to given links.
            visual_name_filter (set or list): If visual_name_filter are provided,
                                              randomization will only apply to given visuals.
            color_range (dict): min-max of each color component (r, g, b).
                Valid format: {'r': {'min':0.0, 'max':1.0},
                               'g': {'min':0.0, 'max':1.0},
                               'b': {'min':0.0, 'max':1.0}}
        """
        super(ModelVisualRandomizer, self).__init__()
        self._model_name = model_name
        self._model_randomizer_type = model_randomizer_type
        self._num_selection = num_selection

        self._color_range = {ColorAttr.R: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                             ColorAttr.G: {RANGE_MIN: 0.0, RANGE_MAX: 1.0},
                             ColorAttr.B: {RANGE_MIN: 0.0, RANGE_MAX: 1.0}}
        if color_range:
            self._color_range.update(color_range)

        # ROS Services Setup
        get_model_prop = ServiceProxyWrapper(GazeboServiceName.GET_MODEL_PROPERTIES, GetModelProperties)
        get_visual_names = ServiceProxyWrapper(GazeboServiceName.GET_VISUAL_NAMES, GetVisualNames)

        # Get all model's link names
        body_names = get_model_prop(GetModelPropertiesRequest(model_name=self._model_name)).body_names
        link_names = ["%s::%s" % (model_name, b) for b in body_names]

        # Convert filters to sets
        link_name_filter = set(link_name_filter) if link_name_filter is not None else None
        visual_name_filter = set(visual_name_filter) if visual_name_filter is not None else None

        if link_name_filter is not None:
            # If link_name_filter is not None then grab the link_name that is in link_name_filter only.
            link_names = [link_name for link_name in link_names if link_name in link_name_filter]

        self._link_visuals_map = {}
        res = get_visual_names(GetVisualNamesRequest(link_names=link_names))
        for idx, visual_name in enumerate(res.visual_names):
            if visual_name_filter is not None and visual_name not in visual_name_filter:
                continue
            link_name = res.link_names[idx]
            if link_name not in self._link_visuals_map:
                self._link_visuals_map[link_name] = []
            self._link_visuals_map[link_name].append(visual_name)

        # rospy.loginfo('link_visuals_map: {}'.format({"model_name:": self._model_name,
        #                                                  "links": self._link_visuals_map}))

    @property
    def model_name(self) -> str:
        """
        Returns the model name.

        Returns:
            str: the model name.

        """
        return self._model_name

    @property
    def model_randomizer_type(self) -> ModelRandomizerType:
        """
        Returns the model randomizer type (MODEL|LINK|VISUAL).

        Returns:
            ModelRandomizerType: the model randomizer type.
        """
        return self._model_randomizer_type

    @model_randomizer_type.setter
    def model_randomizer_type(self, value: ModelRandomizerType) -> None:
        """
        Sets the model randomizer type (MODEL|LINK|VISUAL)

        Args:
            value (ModelRandomizerType): the model randomizer type
        """
        self._model_randomizer_type = value

    @property
    def num_selection(self) -> int:
        """
        Returns the number of selections per randomize.

        Returns:
            int: the number of selections per randomize.
        """
        return self._num_selection

    @num_selection.setter
    def num_selection(self, value: int) -> None:
        """
        Sets the number of selections per randomize.
        - < 0 means all selection.

        Args:
            value (int):  the number of selections per randomize. (>= -1)
        """
        self._num_selection = value

    @property
    def color_range(self) -> dict:
        """
        Returns the copy of color range in dict

        Returns:
            dict: the copy of color range in dict
        """
        return self._color_range.copy()

    @color_range.setter
    def color_range(self, value: dict) -> None:
        """
        Sets the color range given.

        Args:
            value (dict): the color range given.
        """
        self._color_range.update(value)

    def _get_random_color(self) -> Color:
        """
        Return random color

        Returns:
            ColorAttr: generated random color.
        """
        return Color(*[np.random.uniform(self._color_range[ColorAttr.R][RANGE_MIN],
                                         self._color_range[ColorAttr.R][RANGE_MAX]),
                       np.random.uniform(self._color_range[ColorAttr.G][RANGE_MIN],
                                         self._color_range[ColorAttr.G][RANGE_MAX]),
                       np.random.uniform(self._color_range[ColorAttr.B][RANGE_MIN],
                                         self._color_range[ColorAttr.B][RANGE_MAX]),
                       1.0])

    def _randomize(self) -> None:
        """
        Randomize
        """
        link_names = self._link_visuals_map.keys()
        # Unroll all visual names
        visual_names = [visual_name for visual_names in self._link_visuals_map.values()
                        for visual_name in visual_names]

        if self.model_randomizer_type == ModelRandomizerType.LINK and self.num_selection > 0:
            # Select links to randomize if model_randomizer_type is ModelRandomizerType.LINK
            link_names = np.random.choice(list(self._link_visuals_map.keys()),
                                          size=self.num_selection,
                                          replace=False)
        elif self.model_randomizer_type == ModelRandomizerType.VISUAL and self.num_selection > 0:
            # Select visuals to randomize if model_randomizer_type is ModelRandomizerType.VISUAL
            visual_names = np.random.choice(list(visual_names),
                                            size=self.num_selection,
                                            replace=False)
        # Convert to set
        visual_names = set(visual_names)
        # Model-level random color
        color = self._get_random_color()
        material = Material()
        material.ambient = color
        material.diffuse = color
        material.specular = Color(0.0, 0.0, 0.0, 1.0)
        material.emissive = Color(0.0, 0.0, 0.0, 1.0)

        for link_name in link_names:
            for idx, visual_name in enumerate(self._link_visuals_map[link_name]):
                if visual_name not in visual_names:
                    continue
                SetVisualMaterialTracker.get_instance().set_visual_material(link_name=link_name,
                                                                            visual_name=visual_name,
                                                                            material=material)

                if self.model_randomizer_type == ModelRandomizerType.VISUAL:
                    # Visual-level random color
                    color = self._get_random_color()
                    material.ambient = color
                    material.diffuse = color

            if self.model_randomizer_type == ModelRandomizerType.LINK:
                # Link-level random color
                color = self._get_random_color()
                material.ambient = color
                material.diffuse = color
