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
"""A class for abstract effect."""
import abc
from threading import RLock

from deepsim.visual_effects.effect_manager import EffectManager
from deepsim.exception import DeepSimCallbackError

from rosgraph_msgs.msg import Clock


# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class EffectObserverInterface(object):
    """
    Effect Observer Interface
    """
    def on_attach_effect(self, effect: 'AbstractEffect'):
        """
        Callback when effect is attached.

        Args:
            effect (AbstractEffect): effect attached.
        """
        pass

    def on_detach_effect(self, effect: 'AbstractEffect'):
        """
        Callback when effect is detached.

        Args:
            effect (AbstractEffect): effect detached.
        """
        pass


class AbstractEffect(ABC):
    """
    Abstract Effect class

    Effect Call order:
    - attach -> [add to effect_manager] -> on_attach
    After effect is attached to effect manager, effect_manager will call update asynchronously
    - update -> (_lazy_init) -> _update
      - (_laze_init) is called only once at initial update call
    - detach -> [remove from effect_manager] -> on_detach
    """
    def __init__(self) -> None:
        """
        Initialize Abstract Effect class
        """
        self._is_initialized = False
        self._observer_lock = RLock()
        self._lock = RLock()
        self._observers = set()
        self._is_attached = False

    @property
    def is_initialized(self):
        """
        Returns the flag whether instance is initialized or not

        Returns:
            bool: the flag whether instance is initialized or not
        """
        return self._is_initialized

    @property
    def is_in_effect(self):
        """
        Returns the flag whether visual effect is in effect.

        Returns:
            bool: the flag whether visual effect is in effect or not
        """
        return self._is_attached

    def register(self, observer: EffectObserverInterface) -> None:
        """
        Register given observer.

        Args:
            observer (EffectObserverInterface): observer
        """
        with self._observer_lock:
            self._observers.add(observer)

    def unregister(self, observer: EffectObserverInterface) -> None:
        """
        Unregister given observer.

        Args:
            observer (EffectObserverInterface): observer to discard
        """
        with self._observer_lock:
            self._observers.discard(observer)

    def attach(self) -> None:
        """
        Attach the effect to effect manager.
        """
        with self._lock:
            if self._is_attached:
                return
            EffectManager.get_instance().add(self)

            self._is_attached = True
            self.on_attach_effect()
            with self._observer_lock:
                observers = self._observers.copy()

            try:
                for observer in observers:
                    observer.on_attach_effect(self)
            except Exception:
                raise DeepSimCallbackError()

    def detach(self) -> None:
        """
        Detach the effect from effect manager.
        """
        with self._lock:
            if not self._is_attached:
                return

            EffectManager.get_instance().discard(self)

            self._is_attached = False
            self.on_detach_effect()
            with self._observer_lock:
                observers = self._observers.copy()

            try:
                for observer in observers:
                    observer.on_detach_effect(self)
            except Exception:
                raise DeepSimCallbackError()

    def update(self, delta_time: float, sim_time: Clock) -> None:
        """
        Update the effect

        Args:
            delta_time (float): time diff from last call
            sim_time (Clock): simulation time
        """
        if not self._is_initialized:
            self._is_initialized = True
            self._lazy_init()
        self.on_update_effect(delta_time, sim_time)

    def on_attach_effect(self) -> None:
        """
        Subclass should override this if any action needed during attach.
        """
        pass

    def on_detach_effect(self) -> None:
        """
        Subclass should override this if any action needed during detach.
        """
        pass

    def _lazy_init(self) -> None:
        """
        Subclass should override this to do lazy-initialize just before first update call.
        """
        pass

    @abc.abstractmethod
    def on_update_effect(self, delta_time: float, sim_time: Clock) -> None:
        """
        Subclass must override this to update the effect.

        Args:
            delta_time (float): time diff from last call
            sim_time (Clock): simulation time
        """
        raise NotImplementedError('Effect must implement this function')
