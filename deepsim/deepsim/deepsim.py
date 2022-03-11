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
"""A class for DeepSim."""
import threading


class DeepSim(object):
    """DeepSim class"""
    _instance = None
    _instance_lock = threading.RLock()

    @staticmethod
    def get_instance() -> 'DeepSim':
        """
        Method for getting a reference to the DeepSim object

        Returns:
            DeepSim: DeepSim instance
        """
        with DeepSim._instance_lock:
            if DeepSim._instance is None:
                DeepSim()
            return DeepSim._instance

    def __init__(self, is_singleton=True) -> None:
        """
        Initialize DeepSim class

        Args:
            is_singleton (bool): flag whether to instantiate as singleton or not (Should only be used by unit-test)
        """
        if is_singleton:
            if DeepSim._instance is not None:
                raise RuntimeError("Attempting to construct multiple DeepSim")
            DeepSim._instance = self
        # Update the simulator as quickly as possible.
        self._timestep = 0.0

    def start(self):
        """
        Stop the simulation.
        """
        from deepsim.sim_trackers.tracker_manager import TrackerManager
        TrackerManager.get_instance().start()

    def stop(self):
        """
        Start the simulation.
        """
        from deepsim.sim_trackers.tracker_manager import TrackerManager
        TrackerManager.get_instance().stop()

    def pause(self) -> None:
        """
        Pause the simulation.
        """
        from deepsim.sim_trackers.tracker_manager import TrackerManager
        from deepsim.ros.ros_util import ROSUtil
        TrackerManager.get_instance().pause()
        ROSUtil.pause_physics()

    def resume(self) -> None:
        """
        Resume the simulation.
        """
        from deepsim.sim_trackers.tracker_manager import TrackerManager
        from deepsim.ros.ros_util import ROSUtil
        TrackerManager.get_instance().resume()
        ROSUtil.unpause_physics()

    @property
    def timestep(self) -> float:
        """
        Returns the timestep for Simulator update.

        Returns:
            float: timestep for Simulator update.
        """
        return self._timestep

    @timestep.setter
    def timestep(self, value: float) -> None:
        """
        Set Simulator update timestep

        Args:
            value (float): Simulator update timestep
        """
        self._timestep = value
