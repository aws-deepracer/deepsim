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
"""A class for tracker manager."""
from collections import defaultdict
import threading

from deepsim.sim_trackers.tracker import TrackerInterface
import deepsim.sim_trackers.constants as consts
from deepsim.deepsim import DeepSim
from deepsim.exception import DeepSimCallbackError

import rospy
from rosgraph_msgs.msg import Clock


class TrackerManager(object):
    """
    TrackerManager class
    """
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'TrackerManager':
        """
        Method for getting a reference to the Tracker Manager object

        Returns:
            TrackerManager: TrackerManager instance
        """
        with TrackerManager._instance_lock:
            if TrackerManager._instance is None:
                TrackerManager()
            return TrackerManager._instance

    def __init__(self, is_singleton=True):
        """
        Initialize TrackerManager.
        Args:
            is_singleton (bool): the flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if TrackerManager._instance is not None:
                raise RuntimeError("Attempting to construct multiple TrackerManager")
            TrackerManager._instance = self

        self._priority_order = [consts.TrackerPriority.HIGH, consts.TrackerPriority.NORMAL, consts.TrackerPriority.LOW]
        self._tracker_map = defaultdict(set)
        self._lock = threading.RLock()
        self._last_time = 0.0
        self._sim_time = None
        self._callback_event = threading.Event()
        self._should_stop_update = False
        self._update_loop_thread = None
        self._is_paused = False
        rospy.Subscriber('/clock', Clock, self._update_sim_time)

    @property
    def is_paused(self) -> bool:
        """
        Returns the flag whether update loop is in paused state or not.

        Returns:
            bool: the flag whether update loop is in paused state or not.
        """
        return self._is_paused

    def add(self, tracker: TrackerInterface,
            priority: consts.TrackerPriority = consts.TrackerPriority.NORMAL) -> None:
        """
        Add given tracker to manager

        Args:
            tracker (TrackerInterface): tracker object
            priority (consts.TrackerPriority): prioirity
        """
        with self._lock:
            self._tracker_map[priority].add(tracker)

    def remove(self, tracker: TrackerInterface,
               priority: consts.TrackerPriority = consts.TrackerPriority.NORMAL) -> None:
        """
        Remove given tracker from manager
        - If tracker doesn't exist, then KeyError will be raised.

        Args:
            tracker (TrackerInterface): tracker
            priority (consts.TrackerPriority): prioirity
        """
        with self._lock:
            self._tracker_map[priority].remove(tracker)

    def discard(self, tracker: TrackerInterface) -> None:
        """
        Discard given tracker from manager
        - Even tracker doesn't exist, there will be no error raised.

        Args:
            tracker (TrackerInterface): tracker
        """
        with self._lock:
            for priority in self._priority_order:
                self._tracker_map[priority].discard(tracker)

    def start(self) -> None:
        """
        Start the update loop.
        """
        if not self._update_loop_thread:
            self._last_time = None
            self._callback_event.clear()
            self._should_stop_update = False
            self._update_loop_thread = threading.Thread(target=self._update_loop)
            self._update_loop_thread.start()
        else:
            rospy.loginfo("[TrackerManager] Update loop is already running")

    def stop(self) -> None:
        """
        Stop the update loop.
        """
        if self._update_loop_thread:
            self._should_stop_update = True
            self._callback_event.set()
            self._update_loop_thread.join()
            self._update_loop_thread = None

    def pause(self) -> None:
        """
        Pause the update loop.
        """
        self._is_paused = True

    def resume(self) -> None:
        """
        Resume the update loop.
        """
        self._is_paused = False

    def _update_sim_time(self, sim_time: Clock) -> None:
        """
        Callback when sim time is updated

        Args:
            sim_time (Clock): simulation time
        """
        if self._is_paused:
            return
        self._sim_time = sim_time
        self._callback_event.set()

    def _update_loop(self) -> None:
        while not self._should_stop_update:
            self._callback_event.wait()
            self._callback_event.clear()
            if self._should_stop_update:
                break
            sim_time = self._sim_time
            curr_time = sim_time.clock.secs + 1.e-9 * sim_time.clock.nsecs
            if self._last_time is None:
                self._last_time = curr_time
            delta_time = curr_time - self._last_time
            sim_timestep = DeepSim.get_instance().timestep
            if sim_timestep <= 0.0 or delta_time >= sim_timestep:
                # If configured simulator timestep is smaller than or equal to 0.0 means update as quickly as it can.
                # If configured simulator timestep is greater than 0.0, then update the simulator if the delta_time is
                # greater than or equal to configured simulator timestep.
                lock_acquired = self._lock.acquire(False)
                if lock_acquired:
                    try:
                        self._last_time = curr_time
                        for priority in self._priority_order:
                            copy_trackers = self._tracker_map[priority].copy()
                            for tracker in copy_trackers:
                                tracker.on_update_tracker(delta_time, sim_time)
                    except Exception as e:
                        raise DeepSimCallbackError("Tracker raised Exception: {}".format(e))
                    finally:
                        self._lock.release()
                else:
                    rospy.loginfo("[TrackerManager] missed an _update_sim_time call")
