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
"""A class for dummy spawner."""
from typing import Optional

from deepsim.spawners.abs_model_spawner import AbstractModelSpawner
from deepsim.core.pose import Pose


class DummySpawner(AbstractModelSpawner):
    """
    Dummy Spawner class to handle dummy model spawn and delete
    """
    def __init__(self):
        """
        Constructor for DummySpawner
        """
        super().__init__(should_validate_spawn=False,
                         should_validate_delete=False)

    def _spawn(self, model_name: str, pose: Optional[Pose] = None, **kwargs) -> None:
        """
        Spawn dummy model in gazebo simulator

        Args:
            model_name (str): name of the model.
            pose (Optional[Pose]): model pose
            **kwargs: Arbitrary keyword arguments
        """
        pass

    def _delete(self, model_name: str, **kwargs) -> None:
        """
        Delete the non-existing dummy model

        Args:
            model_name (str): name of the model.
            **kwargs: Arbitrary keyword arguments
        """
        pass
