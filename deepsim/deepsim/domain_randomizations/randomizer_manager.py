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
"""A class for randomizer manager."""
import copy
from threading import RLock
from deepsim.domain_randomizations.abs_randomizer import AbstractRandomizer


class RandomizerManager(object):
    """
    Randomizer Manager class that manages multiple randomizer
    """
    _instance = None
    _instance_lock = RLock()

    @staticmethod
    def get_instance() -> 'RandomizerManager':
        """
        Method for getting a reference to the camera manager object

        Returns:
            RandomizerManager: RandomizerManager instance
        """
        with RandomizerManager._instance_lock:
            if RandomizerManager._instance is None:
                RandomizerManager()
            return RandomizerManager._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize Randomizer Manager

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if RandomizerManager._instance:
                raise RuntimeError("Attempting to construct multiple Randomizer Manager")
            # there should be only one randomizer manager instance
            RandomizerManager._instance = self

        self._randomizers = set()
        self._lock = RLock()

    def add(self, randomizer: AbstractRandomizer) -> None:
        """
        Add new randomizer to manager

        Args:
            randomizer (AbstractRandomizer): randomizer
        """
        with self._lock:
            self._randomizers.add(randomizer)

    def remove(self, randomizer: AbstractRandomizer) -> None:
        """
        Remove given randomizer from manager
        - If randomizer doesn't exist, then KeyError will be raised.

        Args:
            randomizer (AbstractRandomizer): randomizer to remove
        """
        with self._lock:
            self._randomizers.remove(randomizer)

    def discard(self, randomizer: AbstractRandomizer) -> None:
        """
        Discard given randomizer from manager
        - Even randomizer doesn't exist, there will be no error raised.

        Args:
            randomizer (AbstractRandomizer): randomizer to discard
        """
        with self._lock:
            self._randomizers.discard(randomizer)

    def randomize(self) -> None:
        """
        Randomize
        """
        with self._lock:
            randomizers = copy.copy(self._randomizers)
        for randomizer in randomizers:
            randomizer.randomize()
