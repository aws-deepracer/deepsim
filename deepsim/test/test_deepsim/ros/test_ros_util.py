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

from deepsim.ros.ros_util import ROSUtil
from deepsim.exception import DeepSimException

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class ROSUtilTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_is_model_spawned(self):
        with patch("deepsim.ros.ros_util.GetModelStateTracker") as get_model_state_tracker_mock:
            assert ROSUtil.is_model_spawned(myself())
            get_model_state_tracker_mock.get_instance.return_value.get_model_state.side_effect = DeepSimException()
            assert not ROSUtil.is_model_spawned(myself())

    def test_wait_for_model_spawn(self):
        with patch("deepsim.ros.ros_util.GetModelStateTracker") as get_model_state_tracker_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            ROSUtil.wait_for_model_spawn(model_name=myself())
            get_model_state_tracker_mock.get_instance.return_value.get_model_state.assert_called_once_with(name=myself(),
                                                                                                           blocking=True)
            time_mock.sleep.assert_not_called()

    def test_wait_for_model_spawn_error(self):
        with patch("deepsim.ros.ros_util.GetModelStateTracker") as get_model_state_tracker_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            get_model_state_tracker_mock.get_instance.return_value.get_model_state.side_effect = DeepSimException()
            with self.assertRaises(DeepSimException):
                ROSUtil.wait_for_model_spawn(model_name=myself(),
                                             max_retry_attempts=3,
                                             backoff_time_sec=0.5)
            assert get_model_state_tracker_mock.get_instance.return_value.get_model_state.call_count == 1 + 3
            time_mock.sleep.has_calls(
                call(0.5), call(0.5), call(0.5), call(0.5), call(0.5)
            )

    def test_wait_for_model_delete(self):
        with patch("deepsim.ros.ros_util.GetModelStateTracker") as get_model_state_tracker_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            get_model_state_tracker_mock.get_instance.return_value.get_model_state.side_effect = DeepSimException()
            ROSUtil.wait_for_model_delete(model_name=myself())
            get_model_state_tracker_mock.get_instance.return_value.get_model_state.assert_called_once_with(name=myself(),
                                                                                                           blocking=True)
            time_mock.sleep.assert_not_called()

    def test_wait_for_model_delete_error(self):
        with patch("deepsim.ros.ros_util.GetModelStateTracker") as get_model_state_tracker_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            with self.assertRaises(DeepSimException):
                ROSUtil.wait_for_model_delete(model_name=myself(),
                                              max_retry_attempts=3,
                                              backoff_time_sec=0.5)
            assert get_model_state_tracker_mock.get_instance.return_value.get_model_state.call_count == 1 + 3
            time_mock.sleep.has_calls(
                call(0.5), call(0.5), call(0.5), call(0.5), call(0.5)
            )

    def test_is_ros_node_alive(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            assert ROSUtil.is_ros_node_alive(myself())
            assert not ROSUtil.is_ros_node_alive(myself() + "_ghost")

    def test_wait_for_rosnode_alive_nodes(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            ROSUtil.wait_for_rosnode(alive_nodes=[myself()])
            assert rosnode_mock.get_node_names.call_count == 1
            time_mock.sleep.assert_not_called()

    def test_wait_for_rosnode_alive_nodes_error(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            node_name = myself() + "_ghost"
            with self.assertRaises(DeepSimException):
                ROSUtil.wait_for_rosnode(alive_nodes=[node_name],
                                         max_retry_attempts=3,
                                         backoff_time_sec=0.5)
            assert rosnode_mock.get_node_names.call_count == 1 + 3
            time_mock.sleep.has_calls(
                call(0.5), call(0.5), call(0.5), call(0.5), call(0.5)
            )

    def test_wait_for_rosnode_dead_nodes(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            node_name = myself() + "_dead"
            ROSUtil.wait_for_rosnode(dead_nodes=[node_name])
            assert rosnode_mock.get_node_names.call_count == 1
            time_mock.sleep.assert_not_called()

    def test_wait_for_rosnode_dead_nodes_error(self):
        with patch("deepsim.ros.ros_util.rosnode") as rosnode_mock, \
                patch("deepsim.ros.ros_util.time") as time_mock:
            rosnode_mock.get_node_names.return_value = [myself()]
            node_name = myself()
            with self.assertRaises(DeepSimException):
                ROSUtil.wait_for_rosnode(dead_nodes=[node_name],
                                         max_retry_attempts=3,
                                         backoff_time_sec=0.5)
            assert rosnode_mock.get_node_names.call_count == 1 + 3
            time_mock.sleep.has_calls(
                call(0.5), call(0.5), call(0.5), call(0.5), call(0.5)
            )
