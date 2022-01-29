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
"""A class for behaviour manager."""
import copy
from collections import defaultdict, OrderedDict
from typing import Union, Any, Optional, Set
from threading import RLock
import threading

from deepsim.behaviours.deepsim_behaviour import DeepSimBehaviour
from deepsim.sim_trackers.tracker import TrackerInterface
from deepsim.sim_trackers.tracker_manager import TrackerManager
import deepsim.sim_trackers.constants as consts

from rosgraph_msgs.msg import Clock


class BehaviourManager(TrackerInterface):
    """
    Behaviour Manager class that manages multiple behaviours
    """
    _instance = None
    _instance_lock = RLock()

    @staticmethod
    def get_instance() -> 'BehaviourManager':
        """
        Method for getting a reference to the camera manager object

        Returns:
            BehaviourManager: BehaviourManager instance
        """
        with BehaviourManager._instance_lock:
            if BehaviourManager._instance is None:
                BehaviourManager()
            return BehaviourManager._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize Behaviour Manager

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if BehaviourManager._instance is not None:
                raise RuntimeError("Attempting to construct multiple Behaviour Manager")
            # there should be only one behaviour manager instance
            BehaviourManager._instance = self

        self._behaviour_map = OrderedDict()
        self._tag_map = defaultdict(set)
        self._behaviour_map_lock = RLock()
        self._tag_map_lock = RLock()

        TrackerManager.get_instance().add(tracker=self, priority=consts.TrackerPriority.NORMAL)

    def get(self, name: str, default: Any = None) -> Union[DeepSimBehaviour, Any]:
        """
        Return behaviour with given name.

        Args:
            name (str): the behaviour's name
            default (Any): default value

        Returns:
            Union[DeepSimBehaviour, Any]: the behaviour with given name
        """
        with self._behaviour_map_lock:
            return self._behaviour_map.get(name, default)

    def find_by_tag(self, tag: str) -> Set[DeepSimBehaviour]:
        """
        Returns behaviours with given tag

        Args:
            tag (str): tag to find the behaviours

        Returns:
            set: the behaviours with given tag.
        """
        with self._tag_map_lock:
            return copy.copy(self._tag_map[tag])

    def add(self, behaviour: DeepSimBehaviour) -> None:
        """
        Add new behaviour to manager

        Args:
            behaviour (DeepSimBehaviour): behaviour
        """
        with self._behaviour_map_lock:
            self._behaviour_map[behaviour.name] = behaviour
        tags = behaviour.tags
        if tags:
            with self._tag_map_lock:
                for tag in tags:
                    self._tag_map[tag].add(behaviour)

    def remove(self, behaviour: Union[str, DeepSimBehaviour]) -> None:
        """
        Remove given behaviour from manager
        - If behaviour doesn't exist, then KeyError will be raised.

        Args:
            behaviour (Union[str, DeepSimBehaviour]): behaviour to remove
        """
        with self._behaviour_map_lock:
            key = behaviour
            if not isinstance(behaviour, str):
                key = behaviour.name
            behaviour = self._behaviour_map.get(key, None)
            del self._behaviour_map[key]
        tags = behaviour.tags if behaviour else None
        if tags:
            with self._tag_map_lock:
                for tag in tags:
                    self._tag_map[tag].discard(behaviour)

    def discard(self, behaviour: Union[str, DeepSimBehaviour]) -> None:
        """
        Discard given behaviour from manager
        - Even behaviour doesn't exist, there will be no error raised.

        Args:
            behaviour (Union[str, DeepSimBehaviour]): behaviour to discard
        """
        with self._behaviour_map_lock:
            key = behaviour
            if not isinstance(behaviour, str):
                key = behaviour.name
            behaviour = self._behaviour_map.pop(key, None)
        tags = behaviour.tags if behaviour else None
        if tags:
            with self._tag_map_lock:
                for tag in tags:
                    self._tag_map[tag].discard(behaviour)

    def update(self, tag: Optional[str] = None) -> None:
        """
        Update the behaviours in manager
        - if tag is provided then operate on the behaviour with given tag. Otherwise, operate on all behaviours.

        Args:
            tag (Optional[Iterable[str]]): tag
        """
        if tag:
            with self._tag_map_lock:
                behaviours = copy.copy(self._tag_map[tag])
            for behaviour in behaviours:
                behaviour.update()
        else:
            with self._behaviour_map_lock:
                behaviour_map = copy.copy(self._behaviour_map)
            for behaviour in behaviour_map.values():
                behaviour.update()

    def reset(self, tag: Optional[str] = None) -> None:
        """
        Reset the behaviours in manager
        - if tag is provided then operate on the behaviour with given tag. Otherwise, operate on all behaviours.

        Args:
            tag (Optional[Iterable[str]]): tag
        """
        if tag:
            with self._tag_map_lock:
                behaviours = copy.copy(self._tag_map[tag])
            for behaviour in behaviours:
                behaviour.reset()
        else:
            with self._behaviour_map_lock:
                behaviour_map = copy.copy(self._behaviour_map)
            for behaviour in behaviour_map.values():
                behaviour.reset()

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update the behaviour on simulator tick.

        Args:
            delta_time (float): delta time
            sim_time (Clock): simulation time
        """
        with self._behaviour_map_lock:
            behaviour_map = copy.copy(self._behaviour_map)
        for behaviour in behaviour_map.values():
            behaviour.fixed_update(delta_time=delta_time, sim_time=sim_time)
