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
"""A class for behaviour's transform."""
from typing import Optional, TypeVar, Iterable, Tuple, Any, List, Collection, Dict, Union
from threading import RLock

from deepsim.sim_trackers.trackers.get_visual_tracker import GetVisualTracker
from deepsim.sim_trackers.trackers.get_model_state_tracker import GetModelStateTracker
from deepsim.sim_trackers.trackers.set_model_state_tracker import SetModelStateTracker
from deepsim.sim_trackers.trackers.set_visual_material_tracker import SetVisualMaterialTracker
from deepsim.sim_trackers.trackers.set_visual_transparency_tracker import SetVisualTransparencyTracker
from deepsim.sim_trackers.trackers.set_visual_visible_tracker import SetVisualVisibleTracker
from deepsim.ros.ros_util import ROSUtil
from deepsim.math.vector3 import Vector3
from deepsim.math.material import Material
from deepsim.math.model_state import ModelState
from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.exception import DeepSimException

from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from deepsim_msgs.srv import (
    GetVisualNames, GetVisualNamesRequest,
    GetVisuals, GetVisualsRequest, GetVisualsResponse
)

DeepSimBehaviour = TypeVar('DeepSimBehaviour')


class Transform:
    """
    Transform class
    """
    lock = RLock()
    transforms = dict()

    def __init__(self,
                 behaviour: DeepSimBehaviour,
                 name: str) -> None:
        """
        Initialize Transform class

        Args:
            behaviour (DeepSimBehaviour): the behaviour object
            name (str): the name of the model
        """
        self._name = name
        self._behaviour = behaviour

        self._get_model_prop_srv = ServiceProxyWrapper(GazeboServiceName.GET_MODEL_PROPERTIES, GetModelProperties)
        self._get_visual_names_srv = ServiceProxyWrapper(GazeboServiceName.GET_VISUAL_NAMES, GetVisualNames)
        # self._get_visuals_srv = ServiceProxyWrapper(GazeboServiceName.GET_VISUALS, GetVisuals)

        self._link_names, self._visual_names = [], []
        self._get_visual_names()

        self._local_state = ModelState(model_name=name)
        self._local_materials = {(link_name, visual_name): Material()
                                 for link_name, visual_name in zip(self._link_names, self._visual_names)}
        self._local_visibles = {(link_name, visual_name): False
                                for link_name, visual_name in zip(self._link_names, self._visual_names)}
        self._local_transparencies = {(link_name, visual_name): 0.0
                                      for link_name, visual_name in zip(self._link_names, self._visual_names)}
        self._material_lock = RLock()
        self._visible_lock = RLock()
        self._transparency_lock = RLock()

        with self.lock:
            Transform.transforms[name] = self

    @property
    def behaviour(self) -> DeepSimBehaviour:
        """
        Returns behaviour object

        Returns:
            DeepSimBehaviour: the behaviour object representing this transform.
        """
        return self._behaviour

    @property
    def name(self) -> str:
        """
        Returns the name of the model.

        Returns:
            str: the name of the model.
        """
        return self._name

    @property
    def link_names(self) -> List[str]:
        """
        Returns the link names of the model.

        Returns:
            List[str]: the link names of the model.
        """
        return list(self._link_names)

    @property
    def visual_names(self) -> List[str]:
        """
        Returns the visual names of the model.

        Returns:
            List[str]: the visual names of the model.
        """
        return list(self._visual_names)

    @property
    def forward(self) -> Vector3:
        """
        Returns the forward vector of the transform.

        Returns:
            Vector3: the forward vector of the transform.
        """
        rot = self.get_state().pose.orientation
        return Vector3.forward().rotate(rot)

    @property
    def back(self) -> Vector3:
        """
        Returns the backward vector of the transform.

        Returns:
            Vector3: the backward vector of the transform.
        """
        rot = self.get_state().pose.orientation
        return Vector3.back().rotate(rot)

    @property
    def left(self) -> Vector3:
        """
        Returns the left vector of the transform.

        Returns:
            Vector3: the left vector of the transform.
        """
        rot = self.get_state().pose.orientation
        return Vector3.left().rotate(rot)

    @property
    def right(self) -> Vector3:
        """
        Returns the right vector of the transform.

        Returns:
            Vector3: the right vector of the transform.
        """
        rot = self.get_state().pose.orientation
        return Vector3.right().rotate(rot)

    @property
    def up(self) -> Vector3:
        """
        Returns the up vector of the transform.

        Returns:
            Vector3: the up vector of the transform.
        """
        rot = self.get_state().pose.orientation
        return Vector3.up().rotate(rot)

    @property
    def down(self) -> Vector3:
        """
        Returns the down vector of the transform.

        Returns:
            Vector3: the down vector of the transform.
        """
        rot = self.get_state().pose.orientation
        return Vector3.down().rotate(rot)

    @property
    def state(self) -> ModelState:
        """
        Returns the model state.

        Returns:
            ModelState: the model state.
        """
        return self.get_state()

    @state.setter
    def state(self, value: ModelState) -> None:
        """
        Set the given model state to transform.

        Args:
            value (ModelState): the model state
        """
        self.set_state(model_state=value)

    @property
    def material(self) -> Dict[Tuple[str, str], Material]:
        """
        Returns the material of the model.
        - Assumes first visual represents the model's material.

        Returns:
            Dict[Tuple[str, str], Material]: {(link_name, visual_name): material}.
        """
        return self.get_material()

    @material.setter
    def material(self, value: Union[Material, Dict[Tuple[str, str], Material]]) -> None:
        """
        Sets the model's visual with given material.

        Args:
            value (Union[Material, Dict[Tuple[str, str], Material]]): the material to set.
        """
        if isinstance(value, Material):
            self.set_material(material=value)
        else:
            link_names, visual_names, materials = [], [], []
            for (link_name, visual_name), material in value.items():
                link_names.append(link_name)
                visual_names.append(visual_name)
                materials.append(material)
            self.set_material(material=materials,
                              link_names=link_names,
                              visual_names=visual_names)

    @property
    def transparency(self) -> Dict[Tuple[str, str], float]:
        """
        Returns the transparency of the model.
        - 0.0 is full transparent and 1.0 is opaque.

        Returns:
            Dict[Tuple[str, str], float]: {(link_name, visual_name): transparency}
        """
        return self.get_transparency()

    @transparency.setter
    def transparency(self, value: Union[float, Dict[Tuple[str, str], float]]) -> None:
        """
        Sets the transparency of the model.
        - 0.0 is full transparent and 1.0 is opaque.

        Args:
            value (nion[float, Dict[Tuple[str, str], float]]): the transparency of the model.
        """
        if isinstance(value, float):
            self.set_transparency(transparency=value)
        else:
            link_names, visual_names, transparencies = [], [], []
            for (link_name, visual_name), transparency in value.items():
                link_names.append(link_name)
                visual_names.append(visual_name)
                transparencies.append(transparency)
            self.set_transparency(transparency=transparencies,
                                  link_names=link_names,
                                  visual_names=visual_names)

    @property
    def visible(self) -> Dict[Tuple[str, str], bool]:
        """
        Returns the visibility of the model.

        Returns:
            Dict[Tuple[str, str], bool]: {(link_name, visual_name): visible}
        """
        return self.get_visible()

    @visible.setter
    def visible(self, value: Union[bool, Dict[Tuple[str, str], bool]]) -> None:
        """
        Sets the visibility of the model.

        Args:
            value (Union[bool, Dict[Tuple[str, str], bool]]): the flag whether to make the model visible or not.
        """
        if isinstance(value, bool):
            self.set_visible(visible=value)
        else:
            link_names, visual_names, visibles = [], [], []
            for (link_name, visual_name), visible in value.items():
                link_names.append(link_name)
                visual_names.append(visual_name)
                visibles.append(visible)
            self.set_visible(visible=visibles,
                             link_names=link_names,
                             visual_names=visual_names)

    def set_state(self, model_state: ModelState, blocking: bool = False) -> None:
        """
        Set the given model state to transform.

        Args:
            model_state (ModelState): the model state
            blocking (bool): flag to block or not
        """
        value_copy = model_state.copy()
        value_copy.model_name = self._name
        SetModelStateTracker.get_instance().set_model_state(value_copy,
                                                            blocking=blocking)
        self._local_state = value_copy

    def get_state(self, reference_frame: Optional[str] = None,
                  blocking: bool = False) -> ModelState:
        """
        Return model state from given reference frame.

        Args:
            reference_frame (Optional[str]): reference from
            blocking (bool): flag to block or not

        Returns:
            ModelState: model state from given reference frame.
        """
        try:
            return GetModelStateTracker.get_instance().get_model_state(self._name,
                                                                       reference_frame=reference_frame,
                                                                       blocking=blocking)
        except DeepSimException:
            return self._local_state.copy()

    def set_material(self, material: Union[Material, Collection[Material]], *,
                     link_names: Optional[Collection[str]] = None,
                     visual_names: Optional[Collection[str]] = None,
                     blocking: bool = False) -> None:
        """
        Sets the model's visual with given material.
        * If either of link_names or visual_names are not provided then,
          given material will be applied to all visuals of this model.

        Args:
            material (Union[Material, Collection[Material]]): the material(s) to set.
            link_names (Optional[Collection[str]]): link names to apply given material.
            visual_names (Optional[Collection[str]]): visual names to apply given material.
            blocking (bool): flag to block or not
        """
        with self._material_lock:
            if link_names is not None and visual_names is not None:
                if len(link_names) != len(visual_names):
                    raise ValueError("link_names ({}) and visual_names ({}) must be same size!".format(len(link_names),
                                                                                                       len(visual_names)))
            else:
                link_names, visual_names = self._get_visual_names()

            if isinstance(material, Material):
                materials = [material.copy() for _ in link_names]
            else:
                materials = material
                if len(materials) != len(link_names):
                    raise ValueError("len(material)[{}] must be same size as link_names[{}]!".format(len(materials),
                                                                                                     len(link_names)))

            if link_names and visual_names:
                SetVisualMaterialTracker.get_instance().set_visual_materials(link_names=link_names,
                                                                             visual_names=visual_names,
                                                                             materials=materials,
                                                                             blocking=blocking)
            updated_materials = {(link_name, visual_name): material
                                 for link_name, visual_name, material in zip(link_names, visual_names, materials)}
            self._local_materials.update(updated_materials)

    def get_material(self, blocking: bool = False) -> Dict[Tuple[str, str], Material]:
        """
        Returns the material of the model.
        - Assumes first visual represents the model's material.

        Args:
            blocking (bool): flag to block or not

        Returns:
            Dict[Tuple[str, str], Material]: {(link_name, visual_name): material}
        """
        with self._material_lock:
            visuals = self.get_visuals(blocking=blocking)
            if visuals:
                materials = {key: visual.material
                             for key, visual in visuals.items() if visual is not None}
                self._local_materials = materials
                return materials
            return self._local_materials.copy()

    def set_transparency(self, transparency: Union[float, Collection[float]], *,
                         link_names: Optional[Collection[str]] = None,
                         visual_names: Optional[Collection[str]] = None,
                         blocking: bool = False) -> None:
        """
        Sets the transparency of the model.
        - 0.0 is full transparent and 1.0 is opaque.
        * If either of link_names or visual_names are not provided then,
          given transparency will be applied to all visuals of this model.

        Args:
            transparency (Union[float, Collection[float]]): the transparency of the model.
            link_names (Optional[Collection[str]]): link names to apply given transparency.
            visual_names (Optional[Collection[str]]): visual names to apply given transparency.
            blocking (bool): flag to block or not
        """
        with self._transparency_lock:
            if link_names is not None and visual_names is not None:
                if len(link_names) != len(visual_names):
                    raise ValueError("link_names ({}) and visual_names ({}) must be same size!".format(len(link_names),
                                                                                                       len(visual_names)))
            else:
                link_names, visual_names = self._get_visual_names()

            if isinstance(transparency, float):
                transparencies = [transparency for _ in link_names]
            else:
                transparencies = transparency
                if len(transparencies) != len(link_names):
                    err_msg = "len(transparency)[{}] must be same size as link_names[{}]!".format(len(transparencies),
                                                                                                  len(link_names))
                    raise ValueError(err_msg)

            if link_names and visual_names:
                SetVisualTransparencyTracker.get_instance().set_visual_transparencies(link_names=link_names,
                                                                                      visual_names=visual_names,
                                                                                      transparencies=transparencies,
                                                                                      blocking=blocking)
            updated_transparecies = {(link_name, visual_name): transparency
                                     for link_name, visual_name, transparency in zip(link_names,
                                                                                     visual_names,
                                                                                     transparencies)}
            self._local_transparencies.update(updated_transparecies)

    def get_transparency(self, blocking: bool = False) -> Dict[Tuple[str, str], float]:
        """
        Returns the transparency of the model.
        - 0.0 is full transparent and 1.0 is opaque.

        Args:
            blocking (bool): flag to block or not

        Returns:
            Dict[Tuple[str, str], float]: {(link_name, visual_name): transparency}
        """
        with self._transparency_lock:
            visuals = self.get_visuals(blocking=blocking)
            if visuals:
                transparencies = {key: visual.transparency
                                  for key, visual in visuals.items() if visual is not None}
                self._local_transparencies = transparencies
                return transparencies
            return self._local_transparencies.copy()

    def set_visible(self, visible: Union[bool, Collection[bool]], *,
                    link_names: Optional[Collection[str]] = None,
                    visual_names: Optional[Collection[str]] = None,
                    blocking: bool = False) -> None:
        """
        Sets the visibility of the model.
        * If either of link_names or visual_names are not provided then,
          given visible will be applied to all visuals of this model.

        Args:
            visible (Union[bool, Collection[bool]]): the flag whether to make the model visible or not.
            link_names (Optional[Collection[str]]): link names to apply given visible.
            visual_names (Optional[Collection[str]]): visual names to apply given visible.
            blocking (bool): flag to block or not
        """
        with self._visible_lock:
            if link_names is not None and visual_names is not None:
                if len(link_names) != len(visual_names):
                    raise ValueError("link_names ({}) and visual_names ({}) must be same size!".format(len(link_names),
                                                                                                       len(visual_names)))
            else:
                link_names, visual_names = self._get_visual_names()

            if isinstance(visible, bool):
                visibles = [visible for _ in link_names]
            else:
                visibles = visible
                if len(visibles) != len(link_names):
                    err_msg = "len(visible)[{}] must be same size as link_names[{}]!".format(len(visibles),
                                                                                             len(link_names))
                    raise ValueError(err_msg)
            if link_names and visual_names:
                SetVisualVisibleTracker.get_instance().set_visual_visibles(link_names=link_names,
                                                                           visual_names=visual_names,
                                                                           visibles=visibles,
                                                                           blocking=blocking)

            updated_visibles = {(link_name, visual_name): visible
                                for link_name, visual_name, visible in zip(link_names,
                                                                           visual_names,
                                                                           visibles)}
            self._local_visibles.update(updated_visibles)

    def get_visible(self, blocking: bool = False) -> Dict[Tuple[str, str], bool]:
        """
        Returns the visibility of the model.

        Args:
            blocking (bool): flag to block or not

        Returns:
            Dict[Tuple[str, str], bool]: {(link_name, visual_name): visible}
        """
        with self._visible_lock:
            visuals = self.get_visuals(blocking=blocking)

            if visuals:
                visibles = {key: visual.visible
                            for key, visual in visuals.items() if visual is not None}
                self._local_visibles = visibles
                return visibles
            return self._local_visibles.copy()

    def _get_visual_names(self) -> Tuple[Collection[str], Collection[str]]:
        """
        Returns link and visual names.

        Returns:
            Tuple[Collection[str], Collection[str]]: link and visual names respectively.
        """
        if ROSUtil.is_model_spawned(self._name):
            if not self._link_names or not self._visual_names:
                req = GetModelPropertiesRequest()
                req.model_name = self.name
                body_names = self._get_model_prop_srv(req).body_names

                link_names = ["%s::%s" % (self.name, b) for b in body_names]
                req = GetVisualNamesRequest()
                req.link_names = link_names
                res = self._get_visual_names_srv(req)

                self._link_names, self._visual_names = res.link_names, res.visual_names
        else:
            self._link_names, self._visual_names = [], []
        return self._link_names, self._visual_names

    def get_visuals(self, blocking: bool = False) -> GetVisualsResponse:
        """
        Returns current visual of the model.

        Args:
            blocking (bool): flag to block or not

        Returns:
            Dict[Tuple[str, str], Visual]: {(link_name, visual_name): visual}
        """
        link_names, visual_names = self._get_visual_names()
        if link_names and visual_names:
            return GetVisualTracker.get_instance().get_visuals(link_names=link_names,
                                                               visual_names=visual_names,
                                                               blocking=blocking)
        return {}

    @staticmethod
    def find(name: str, default: Optional[Any] = None) -> 'Transform':
        """
        Find the transform with model name.

        Args:
            name (str): the model name.
            default (Optional[Any]): default value to return if not found.

        Returns:
            Transform: the transform object found.
        """
        with Transform.lock:
            return Transform.transforms.get(name, default)

    @staticmethod
    def delete(name: str) -> None:
        """
        Delete the transform with given model name.

        Args:
            name (str): the model name to delete the transform.
        """
        with Transform.lock:
            if name in Transform.transforms:
                transform = Transform.transforms[name]
                transform._behaviour = None
                Transform.transforms.pop(name, None)
