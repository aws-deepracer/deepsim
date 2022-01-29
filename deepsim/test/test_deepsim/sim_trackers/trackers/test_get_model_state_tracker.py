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
from typing import Any, Callable
from unittest import TestCase
from unittest.mock import patch, MagicMock, call
import inspect

from deepsim.sim_trackers.trackers.get_model_state_tracker import GetModelStateTracker
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.math.pose import Pose
from deepsim.math.twist import Twist
from deepsim.math.vector3 import Vector3
from deepsim.math.quaternion import Quaternion
from deepsim.math.model_state import ModelState
from deepsim.exception import DeepSimException

from deepsim_msgs.srv import (
    GetModelStates, GetModelStatesResponse,
    GetAllModelStates, GetAllModelStatesResponse
)

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.sim_trackers.trackers.get_model_state_tracker.ServiceProxyWrapper")
class GetModelStateTrackerTest(TestCase):
    def setUp(self) -> None:
        get_model_states_mock = MagicMock()
        get_all_model_states_mock = MagicMock()

        def get_service_mock(service_name, service_type):
            if service_name == GazeboServiceName.GET_MODEL_STATES:
                return get_model_states_mock
            elif service_name == GazeboServiceName.GET_ALL_MODEL_STATES:
                return get_all_model_states_mock
            else:
                return Exception()

        self.get_model_states_mock = get_model_states_mock
        self.get_all_model_states_mock = get_all_model_states_mock
        self.get_service_mock = get_service_mock

    def test_initialize(self, service_proxy_wrapper_mock):
        _ = GetModelStateTracker(is_singleton=False)
        service_proxy_wrapper_mock.assert_has_calls([
            call(GazeboServiceName.GET_MODEL_STATES, GetModelStates),
            call(GazeboServiceName.GET_ALL_MODEL_STATES, GetAllModelStates),
        ])

    def test_on_update_tracker(self, service_proxy_wrapper_mock):
        model_name1 = myself() + "1"
        model_name2 = myself() + "2"
        pose1 = Pose(position=Vector3(1.0, 2.0, 3.0),
                     orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        pose2 = Pose(position=Vector3(3.0, 4.0, 5.0),
                     orientation=Quaternion(4.0, 5.0, 6.0, 7.0))
        twist1 = Twist(linear=Vector3(2.0, 3.0, 4.0),
                       angular=Vector3(3.0, 4.0, 5.0))
        twist2 = Twist(linear=Vector3(3.0, 4.0, 5.0),
                       angular=Vector3(4.0, 5.0, 6.0))

        expected_model_state1 = ModelState(model_name=model_name1,
                                           pose=pose1,
                                           twist=twist1)
        expected_model_state2 = ModelState(model_name=model_name2,
                                           pose=pose2,
                                           twist=twist2)
        res = GetAllModelStatesResponse()
        res.success = True
        res.status_message = ''
        res.model_states = [expected_model_state1.to_ros(),
                            expected_model_state2.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_model_states_mock.return_value = res

        tracker = GetModelStateTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        assert model_name1 in tracker._model_map
        assert model_name2 in tracker._model_map
        assert expected_model_state1 == tracker._model_map[model_name1]
        assert expected_model_state2 == tracker._model_map[model_name2]

    def test_get_model_state(self, service_proxy_wrapper_mock):
        model_name1 = myself() + "1"
        model_name2 = myself() + "2"
        pose1 = Pose(position=Vector3(1.0, 2.0, 3.0),
                     orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        pose2 = Pose(position=Vector3(3.0, 4.0, 5.0),
                     orientation=Quaternion(4.0, 5.0, 6.0, 7.0))
        twist1 = Twist(linear=Vector3(2.0, 3.0, 4.0),
                       angular=Vector3(3.0, 4.0, 5.0))
        twist2 = Twist(linear=Vector3(3.0, 4.0, 5.0),
                       angular=Vector3(4.0, 5.0, 6.0))

        expected_model_state1 = ModelState(model_name=model_name1,
                                           pose=pose1,
                                           twist=twist1)
        expected_model_state2 = ModelState(model_name=model_name2,
                                           pose=pose2,
                                           twist=twist2)

        res = GetAllModelStatesResponse()
        res.success = True
        res.status_message = ''
        res.model_states = [expected_model_state1.to_ros(),
                            expected_model_state2.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_model_states_mock.return_value = res

        tracker = GetModelStateTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        model_state1 = tracker.get_model_state(model_name1)
        model_state2 = tracker.get_model_state(model_name2)

        assert model_state1 == expected_model_state1
        assert model_state2 == expected_model_state2

        # Check returned model state is a copy
        assert tracker._model_map[model_name1] == model_state1
        assert tracker._model_map[model_name1] is not model_state1
        assert tracker._model_map[model_name2] == model_state2
        assert tracker._model_map[model_name2] is not model_state2
        self.get_model_states_mock.assert_not_called()

    def test_get_model_state_blocking(self, service_proxy_wrapper_mock):
        model_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        expected_model_state = ModelState(model_name=model_name,
                                          pose=pose,
                                          twist=twist)

        res = GetModelStatesResponse()
        res.status = [True]
        res.success = True
        res.model_states = [expected_model_state.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_model_states_mock.return_value = res

        tracker = GetModelStateTracker(is_singleton=False)
        model_state = tracker.get_model_state(model_name, blocking=True)
        self.get_model_states_mock.assert_called_once_with([model_name], [''])
        assert model_state == expected_model_state

    def test_get_model_state_missing_in_dict(self, service_proxy_wrapper_mock):
        model_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        expected_model_state = ModelState(model_name=model_name,
                                          pose=pose,
                                          twist=twist)

        res = GetModelStatesResponse()
        res.status = [True]
        res.success = True
        res.model_states = [expected_model_state.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_model_states_mock.return_value = res

        tracker = GetModelStateTracker(is_singleton=False)
        model_state = tracker.get_model_state(model_name)
        self.get_model_states_mock.assert_called_once_with([model_name], [''])
        assert model_state == expected_model_state

    def test_get_model_state_missing_and_failed_to_retrieve(self, service_proxy_wrapper_mock):
        model_name = myself()

        res = GetModelStatesResponse()
        res.status = [False]
        res.success = False
        res.model_states = [None]
        res.messages = ["Failed"]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_model_states_mock.return_value = res

        tracker = GetModelStateTracker(is_singleton=False)
        with self.assertRaises(DeepSimException):
            tracker.get_model_state(model_name)

        res.status = [False]
        res.success = True

        with self.assertRaises(DeepSimException):
            tracker.get_model_state(model_name)

        res.status = [True]
        res.success = False

        with self.assertRaises(DeepSimException):
            tracker.get_model_state(model_name)

        self.get_model_states_mock.has_calls(
            call([model_name], ['']),
            call([model_name], ['']),
            call([model_name], [''])
        )

    def test_get_model_state_with_reference_frame(self, service_proxy_wrapper_mock):
        reference_frame = myself() + "_reference_frame"

        model_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))

        expected_model_state = ModelState(model_name=model_name,
                                          pose=pose,
                                          twist=twist)

        res = GetAllModelStatesResponse()
        res.success = True
        res.status_message = ''
        res.model_states = [expected_model_state.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_model_states_mock.return_value = res

        tracker = GetModelStateTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        expected_model_state_w_ref = ModelState(model_name=model_name,
                                                pose=pose,
                                                twist=twist,
                                                reference_frame=reference_frame)

        res = GetModelStatesResponse()
        res.status = [True]
        res.success = True
        res.model_states = [expected_model_state_w_ref.to_ros()]
        self.get_model_states_mock.return_value = res

        model_state = tracker.get_model_state(model_name)

        assert model_state == expected_model_state
        # Check returned model state is a copy
        assert tracker._model_map[model_name] == model_state
        assert tracker._model_map[model_name] is not model_state

        model_state_w_ref = tracker.get_model_state(model_name, reference_frame=reference_frame)
        assert model_state_w_ref == expected_model_state_w_ref
        self.get_model_states_mock.assert_called_once_with([model_name],
                                                           [reference_frame])

    def test_get_model_states(self, service_proxy_wrapper_mock):
        model_name1 = myself() + "1"
        model_name2 = myself() + "2"
        pose1 = Pose(position=Vector3(1.0, 2.0, 3.0),
                     orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        pose2 = Pose(position=Vector3(3.0, 4.0, 5.0),
                     orientation=Quaternion(4.0, 5.0, 6.0, 7.0))
        twist1 = Twist(linear=Vector3(2.0, 3.0, 4.0),
                       angular=Vector3(3.0, 4.0, 5.0))
        twist2 = Twist(linear=Vector3(3.0, 4.0, 5.0),
                       angular=Vector3(4.0, 5.0, 6.0))

        model_state1 = ModelState(model_name=model_name1,
                                  pose=pose1,
                                  twist=twist1)
        model_state2 = ModelState(model_name=model_name2,
                                  pose=pose2,
                                  twist=twist2)
        res = GetAllModelStatesResponse()
        res.success = True
        res.status_message = ''
        res.model_states = [model_state1.to_ros(), model_state2.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_model_states_mock.return_value = res

        tracker = GetModelStateTracker(is_singleton=False)

        tracker.on_update_tracker(0.1, None)

        model_states = tracker.get_model_states([model_name1, model_name2])

        key1 = (model_name1, '')
        key2 = (model_name2, '')
        expected_model_states = {key1: ModelState(model_name=model_name1,
                                                  pose=pose1,
                                                  twist=twist1),
                                 key2: ModelState(model_name=model_name2,
                                                  pose=pose2,
                                                  twist=twist2)}

        assert model_states == expected_model_states

        self.get_model_states_mock.assert_not_called()

    def test_get_model_states_blocking(self, service_proxy_wrapper_mock):
        model_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        expected_model_state = ModelState(model_name=model_name,
                                          pose=pose,
                                          twist=twist)
        key = (model_name, '')
        expected_return = {key: expected_model_state}

        res = GetModelStatesResponse()
        res.status = [True]
        res.success = True
        res.model_states = [expected_model_state.to_ros()]
        self.get_model_states_mock.return_value = res
        service_proxy_wrapper_mock.side_effect = self.get_service_mock

        tracker = GetModelStateTracker(is_singleton=False)
        model_state = tracker.get_model_states([model_name], blocking=True)
        self.get_model_states_mock.assert_called_once_with([model_name], [''])
        assert model_state == expected_return

    def test_get_model_states_missing_in_dict(self, service_proxy_wrapper_mock):
        model_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        expected_model_state = ModelState(model_name=model_name,
                                          pose=pose,
                                          twist=twist)
        key = (model_name, '')
        expected_return = {key: expected_model_state}

        res = GetModelStatesResponse()
        res.status = [True]
        res.success = True
        res.model_states = [expected_model_state.to_ros()]
        self.get_model_states_mock.return_value = res
        service_proxy_wrapper_mock.side_effect = self.get_service_mock

        tracker = GetModelStateTracker(is_singleton=False)
        model_state = tracker.get_model_states([model_name])
        self.get_model_states_mock.assert_called_once_with([model_name], [''])
        assert model_state == expected_return

    def test_get_model_states_missing_and_failed_to_retrieve(self, service_proxy_wrapper_mock):
        model_name = myself()

        res = GetModelStatesResponse()
        res.status = [False]
        res.success = False
        res.model_states = [None]
        res.messages = ["Failed"]

        self.get_model_states_mock.return_value = res
        service_proxy_wrapper_mock.side_effect = self.get_service_mock

        tracker = GetModelStateTracker(is_singleton=False)
        with self.assertRaises(DeepSimException):
            tracker.get_model_states([model_name])

        res.status = [False]
        res.success = True

        assert tracker.get_model_states([model_name]) == {(model_name, ''): None}

        res.status = [True]
        res.success = False

        with self.assertRaises(DeepSimException):
            tracker.get_model_states([model_name])

        self.get_model_states_mock.has_calls(
            call([model_name], ['']),
            call([model_name], ['']),
            call([model_name], [''])
        )

    def test_get_model_states_with_reference_frame(self, service_proxy_wrapper_mock):
        reference_frame = myself() + "_reference_frame"

        model_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))

        expected_model_state = ModelState(model_name=model_name,
                                          pose=pose,
                                          twist=twist)

        res = GetAllModelStatesResponse()
        res.success = True
        res.status_message = ''
        res.model_states = [expected_model_state.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_model_states_mock.return_value = res

        tracker = GetModelStateTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        expected_model_state_w_ref = ModelState(model_name=model_name,
                                                pose=pose,
                                                twist=twist,
                                                reference_frame=reference_frame)
        key = (model_name, reference_frame)
        expected_return = {key: expected_model_state_w_ref}

        res = GetModelStatesResponse()
        res.status = [True]
        res.success = True
        res.model_states = [expected_model_state_w_ref.to_ros()]
        self.get_model_states_mock.return_value = res

        model_state = tracker.get_model_state(model_name)

        assert model_state == expected_model_state
        # Check returned model state is a copy
        assert tracker._model_map[model_name] == model_state
        assert tracker._model_map[model_name] is not model_state

        model_state_w_ref = tracker.get_model_states([model_name], reference_frames=[reference_frame])
        assert model_state_w_ref == expected_return
        self.get_model_states_mock.assert_called_once_with([model_name],
                                                           [reference_frame])

    def test_get_model_states_unmatched_length(self, service_proxy_wrapper_mock):
        tracker = GetModelStateTracker(is_singleton=False)

        model_name = myself()
        with self.assertRaises(ValueError):
            tracker.get_model_states([model_name], [])

    def test_set_model_state(self, service_proxy_wrapper_mock):
        tracker = GetModelStateTracker(is_singleton=False)
        model_name = myself()
        expected_model_state = ModelState(model_name=model_name)
        tracker.set_model_state(expected_model_state)
        assert expected_model_state == tracker.get_model_state(name=model_name)
