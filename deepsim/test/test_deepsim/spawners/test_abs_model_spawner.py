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
from typing import Any, Callable, Optional, Iterable
from unittest import TestCase
from unittest.mock import patch, MagicMock, call
import inspect
from typing import Optional

from deepsim.exception import DeepSimException
from deepsim.math.pose import Pose
from deepsim.math.point import Point
from deepsim.math.quaternion import Quaternion
from deepsim.spawners.abs_model_spawner import AbstractModelSpawner

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class DummyModelSpawner(AbstractModelSpawner):
    def __init__(self,
                 should_validate_spawn: bool = True,
                 should_validate_delete: bool = True):
        """
        Dummy model for testing AbsModel
        """
        super().__init__(should_validate_spawn=should_validate_spawn,
                         should_validate_delete=should_validate_delete)
        self.mock = MagicMock()

    def _spawn(self, model_name: str,
               robot_namespace: str,
               initial_pose: Pose,
               reference_frame: str,
               **kwargs) -> None:
        """
        Spawn model in gazebo simulator

        Args:
            model_name (str): name of the model.
            robot_namespace (str): robot namespace
            initial_pose (Pose): model spawn pose
            reference_frame (str): reference frame
            **kwargs: Arbitrary keyword arguments
        """
        self.mock.spawn(model_name=model_name,
                        robot_namespace=robot_namespace,
                        initial_pose=initial_pose,
                        reference_frame=reference_frame,
                        **kwargs)

    def _delete(self, model_name: str, **kwargs) -> None:
        """Delete model

        Args:
            model_name (str): name of the model.
            **kwargs: Arbitrary keyword arguments
        """
        self.mock.delete(model_name=model_name,
                         **kwargs)


@patch("deepsim.ros.ros_util.GetModelStateTracker")
@patch("deepsim.spawners.abs_model_spawner.rospy")
class AbsModelTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialization(self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test model initialization

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        spawner = DummyModelSpawner()
        assert spawner._should_validate_spawn
        assert spawner._should_validate_delete
        spawner = DummyModelSpawner(should_validate_spawn=False,
                                    should_validate_delete=True)
        assert not spawner._should_validate_spawn
        assert spawner._should_validate_delete

        spawner = DummyModelSpawner(should_validate_spawn=True,
                                    should_validate_delete=False)
        assert spawner._should_validate_spawn
        assert not spawner._should_validate_delete

        spawner = DummyModelSpawner(should_validate_spawn=False,
                                    should_validate_delete=False)
        assert not spawner._should_validate_spawn
        assert not spawner._should_validate_delete

    def test_spawn(self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn model at default pose

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner()
        model_name = myself()

        get_model_state_tracker_mock.get_instance().get_model_state.return_value = None
        dummy_model_spawner.spawn(model_name=model_name)

        expected_robot_namespace = "/{}".format(model_name)
        expected_reference_frame = ""
        dummy_model_spawner.mock.spawn.assert_called_once_with(model_name=model_name,
                                                               robot_namespace=expected_robot_namespace,
                                                               initial_pose=Pose(),
                                                               reference_frame=expected_reference_frame)
        self.assertIn(model_name, AbstractModelSpawner._model_names)

    def test_spawn_with_robot_namespace(self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn model at default pose

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner()
        model_name = myself()
        robot_namespace = myself() + "_robot_namespace"

        get_model_state_tracker_mock.get_instance().get_model_state.return_value = None
        dummy_model_spawner.spawn(model_name=model_name,
                                  robot_namespace=robot_namespace)
        dummy_model_spawner.mock.spawn.assert_called_once_with(model_name=model_name,
                                                               robot_namespace=robot_namespace,
                                                               initial_pose=Pose(),
                                                               reference_frame='')

    def test_spawn_with_reference_frame(self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn model at default pose

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner()
        model_name = myself()
        reference_frame = myself() + "_reference_frame"

        get_model_state_tracker_mock.get_instance().get_model_state.return_value = None
        dummy_model_spawner.spawn(model_name=model_name,
                                  reference_frame=reference_frame)

        expected_robot_namespace = "/{}".format(model_name)

        dummy_model_spawner.mock.spawn.assert_called_once_with(model_name=model_name,
                                                               robot_namespace=expected_robot_namespace,
                                                               initial_pose=Pose(),
                                                               reference_frame=reference_frame)
        self.assertIn(model_name, AbstractModelSpawner._model_names)

    def test_spawn_no_validation(self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn model at default pose

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner(should_validate_spawn=False)
        model_name = myself()

        get_model_state_tracker_mock.get_instance().get_model_state.return_value = None
        with patch("deepsim.spawners.abs_model_spawner.ROSUtil") as ros_util_mock:
            dummy_model_spawner.spawn(model_name=model_name)
            ros_util_mock.wait_for_model_spawn.assert_not_called()
        dummy_model_spawner.mock.spawn.assert_called_once_with(model_name=model_name,
                                                               robot_namespace="/{}".format(model_name),
                                                               initial_pose=Pose(),
                                                               reference_frame='')
        self.assertIn(model_name, AbstractModelSpawner._model_names)

    def test_spawn_other_pose_succeed(self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn model at other pose

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner()
        model_name = myself()
        pose_at_1_1_1 = Pose(Point(1, 1, 1), Quaternion())
        get_model_state_tracker_mock.get_instance().get_model_state.return_value = None
        dummy_model_spawner.spawn(model_name=model_name,
                                  initial_pose=pose_at_1_1_1)
        dummy_model_spawner.mock.spawn.assert_called_once_with(model_name=model_name,
                                                               robot_namespace="/{}".format(model_name),
                                                               initial_pose=pose_at_1_1_1,
                                                               reference_frame='')
        self.assertIn(model_name, AbstractModelSpawner._model_names)

    def test_spawn_get_model_state_fault(self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn model with get model state fault

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner()
        model_name = myself()

        with self.assertRaises(DeepSimException):
            get_model_state_tracker_mock.get_instance().get_model_state.side_effect = DeepSimException("")
            dummy_model_spawner.spawn(model_name=model_name)
        dummy_model_spawner.mock.spawn.assert_called_once_with(model_name=model_name,
                                                               robot_namespace="/{}".format(model_name),
                                                               initial_pose=Pose(),
                                                               reference_frame='')
        self.assertNotIn(model_name, AbstractModelSpawner._model_names)

    def test_spawn_and_spawn_with_same_dummy_model(
            self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn and spawn again with same dummy model but different model name

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner()
        model_name = myself()

        get_model_state_tracker_mock.get_instance().get_model_state.return_value = None
        assert dummy_model_spawner.spawn(model_name=model_name)
        self.assertIn(model_name, AbstractModelSpawner._model_names)

        assert not dummy_model_spawner.spawn(model_name=model_name)

    def test_spawn_and_delete(
            self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn and then delete

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner()
        model_name = myself()
        get_model_state_tracker_mock.get_instance().get_model_state.return_value = None
        dummy_model_spawner.spawn(model_name=model_name)
        dummy_model_spawner.mock.spawn.assert_called_once_with(model_name=model_name,
                                                               robot_namespace="/{}".format(model_name),
                                                               initial_pose=Pose(),
                                                               reference_frame='')
        self.assertIn(model_name, AbstractModelSpawner._model_names)

        get_model_state_tracker_mock.get_instance().get_model_state.side_effect = DeepSimException("")
        dummy_model_spawner.delete(model_name=model_name)
        dummy_model_spawner.mock.delete.assert_called_once()
        self.assertNotIn(model_name, AbstractModelSpawner._model_names)

    def test_spawn_and_delete_no_validation(
            self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn and then delete

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner(should_validate_delete=False)
        model_name = myself()
        get_model_state_tracker_mock.get_instance().get_model_state.return_value = None
        with patch("deepsim.spawners.abs_model_spawner.ROSUtil") as ros_util_mock:
            dummy_model_spawner.spawn(model_name=model_name)
            ros_util_mock.wait_for_model_spawn.assert_called_once_with(model_name=model_name,
                                                                       max_retry_attempts=10,
                                                                       backoff_time_sec=1.0)
        dummy_model_spawner.mock.spawn.assert_called_once_with(model_name=model_name,
                                                               robot_namespace="/{}".format(model_name),
                                                               initial_pose=Pose(),
                                                               reference_frame='')
        self.assertIn(model_name, AbstractModelSpawner._model_names)

        get_model_state_tracker_mock.get_instance().get_model_state.side_effect = DeepSimException("")
        with patch("deepsim.spawners.abs_model_spawner.ROSUtil") as ros_util_mock:
            dummy_model_spawner.delete(model_name=model_name)
            ros_util_mock.wait_for_model_delete.assert_not_called()
        dummy_model_spawner.mock.delete.assert_called_once()
        self.assertNotIn(model_name, AbstractModelSpawner._model_names)

    def test_spawn_and_delete_get_model_state_fault(
            self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test spawn and delete with get model state fault

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        with self.assertRaises(DeepSimException):
            dummy_model_spawner = DummyModelSpawner()
            model_name = myself()
            get_model_state_tracker_mock.get_instance().get_model_state.return_value = None
            dummy_model_spawner.spawn(model_name=model_name)
            dummy_model_spawner.mock.spawn.assert_called_once_with(model_name=model_name,
                                                                   robot_namespace="/{}".format(model_name),
                                                                   initial_pose=Pose(),
                                                                   reference_frame='')
            self.assertIn(model_name, AbstractModelSpawner._model_names)

            dummy_model_spawner.delete(model_name=model_name)

        self.assertIn(model_name, AbstractModelSpawner._model_names)

    def test_delete(
            self, rospy_mock, get_model_state_tracker_mock) -> None:
        """test delete

        Args:
            rospy_mock (Mock): rospy mock
            get_model_state_tracker_mock (Mock): GetModelStateTracker mock
        """
        dummy_model_spawner = DummyModelSpawner()
        model_name = myself()
        assert not dummy_model_spawner.delete(model_name=model_name)
        self.assertNotIn(model_name, AbstractModelSpawner._model_names)
