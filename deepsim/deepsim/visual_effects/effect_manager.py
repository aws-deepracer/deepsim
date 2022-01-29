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
"""A class for effect manager."""
import threading
from typing import TypeVar
from deepsim.sim_trackers.tracker import TrackerInterface
from deepsim.sim_trackers.tracker_manager import TrackerManager
import deepsim.sim_trackers.constants as consts

from rosgraph_msgs.msg import Clock

AbstractEffect = TypeVar('AbstractEffect')


class EffectManager(TrackerInterface):
    """
    Effect Manager class that manages multiple effects
    """
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'EffectManager':
        """
        Method for getting a reference to the effect manager object

        Returns:
            EffectManager: EffectManager instance
        """
        with EffectManager._instance_lock:
            if EffectManager._instance is None:
                EffectManager()
            return EffectManager._instance

    def __init__(self, is_singleton=True):
        """
        Initialize Effect Manager class

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if EffectManager._instance is not None:
                raise RuntimeError("Attempting to construct multiple Effect Manager")
            # there should be only one randomizer manager instance
            EffectManager._instance = self
        self._effects = set()
        self._lock = threading.RLock()

        TrackerManager.get_instance().add(tracker=self, priority=consts.TrackerPriority.NORMAL)

    def on_update_tracker(self, delta_time: float, sim_time: Clock) -> None:
        """
        Callback when sim time is updated

        Args:
            delta_time (float): time diff from last call
            sim_time (Clock): simulation time
        """
        with self._lock:
            copy_effect_set = self._effects.copy()
            for effect in copy_effect_set:
                effect.update(delta_time, sim_time)

    def add(self, effect: AbstractEffect) -> None:
        """
        Add effect to manager

        Args:
            effect (AbstractEffect): the effect to add
        """
        with self._lock:
            self._effects.add(effect)

    def remove(self, effect: AbstractEffect) -> None:
        """
        Remove effect from manager
        - If effect doesn't exist, then KeyError will be raised.

        Args:
            effect (AbstractEffect): the effect to remove
        """
        with self._lock:
            self._effects.remove(effect)

    def discard(self, effect: AbstractEffect) -> None:
        """
        Discard effect from manager
        - Even effect doesn't exist, there will be no error raised.

        Args:
            effect (AbstractEffect): the effect to discard
        """
        with self._lock:
            self._effects.discard(effect)
