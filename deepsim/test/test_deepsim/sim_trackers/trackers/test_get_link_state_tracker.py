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

from deepsim.sim_trackers.trackers.get_link_state_tracker import GetLinkStateTracker
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.math.pose import Pose
from deepsim.math.twist import Twist
from deepsim.math.vector3 import Vector3
from deepsim.math.quaternion import Quaternion
from deepsim.math.link_state import LinkState
from deepsim.exception import DeepSimException

from deepsim_msgs.srv import (
    GetLinkStates, GetLinkStatesResponse,
    GetAllLinkStates, GetAllLinkStatesResponse
)

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.sim_trackers.trackers.get_link_state_tracker.ServiceProxyWrapper")
class GetLinkStateTrackerTest(TestCase):
    def setUp(self) -> None:
        get_link_states_mock = MagicMock()
        get_all_link_states_mock = MagicMock()

        def get_service_mock(service_name, service_type):
            if service_name == GazeboServiceName.GET_LINK_STATES:
                return get_link_states_mock
            elif service_name == GazeboServiceName.GET_ALL_LINK_STATES:
                return get_all_link_states_mock
            else:
                return Exception()
        self.get_link_states_mock = get_link_states_mock
        self.get_all_link_states_mock = get_all_link_states_mock
        self.get_service_mock = get_service_mock

    def test_initialize(self, service_proxy_wrapper_mock):
        _ = GetLinkStateTracker(is_singleton=False)
        service_proxy_wrapper_mock.assert_has_calls([
            call(GazeboServiceName.GET_LINK_STATES, GetLinkStates),
            call(GazeboServiceName.GET_ALL_LINK_STATES, GetAllLinkStates),
        ])

    def test_on_update_tracker(self, service_proxy_wrapper_mock):
        link_name1 = myself() + "1"
        link_name2 = myself() + "2"
        pose1 = Pose(position=Vector3(1.0, 2.0, 3.0),
                     orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        pose2 = Pose(position=Vector3(3.0, 4.0, 5.0),
                     orientation=Quaternion(4.0, 5.0, 6.0, 7.0))
        twist1 = Twist(linear=Vector3(2.0, 3.0, 4.0),
                       angular=Vector3(3.0, 4.0, 5.0))
        twist2 = Twist(linear=Vector3(3.0, 4.0, 5.0),
                       angular=Vector3(4.0, 5.0, 6.0))

        expected_link_state1 = LinkState(link_name=link_name1,
                                         pose=pose1,
                                         twist=twist1)
        expected_link_state2 = LinkState(link_name=link_name2,
                                         pose=pose2,
                                         twist=twist2)
        res = GetAllLinkStatesResponse()
        res.success = True
        res.status_message = ''
        res.link_states = [expected_link_state1.to_ros(),
                           expected_link_state2.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_link_states_mock.return_value = res

        tracker = GetLinkStateTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        assert link_name1 in tracker._link_map
        assert link_name2 in tracker._link_map
        assert expected_link_state1 == tracker._link_map[link_name1]
        assert expected_link_state2 == tracker._link_map[link_name2]

    def test_get_link_state(self, service_proxy_wrapper_mock):
        link_name1 = myself() + "1"
        link_name2 = myself() + "2"
        pose1 = Pose(position=Vector3(1.0, 2.0, 3.0),
                     orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        pose2 = Pose(position=Vector3(3.0, 4.0, 5.0),
                     orientation=Quaternion(4.0, 5.0, 6.0, 7.0))
        twist1 = Twist(linear=Vector3(2.0, 3.0, 4.0),
                       angular=Vector3(3.0, 4.0, 5.0))
        twist2 = Twist(linear=Vector3(3.0, 4.0, 5.0),
                       angular=Vector3(4.0, 5.0, 6.0))

        expected_link_state1 = LinkState(link_name=link_name1,
                                         pose=pose1,
                                         twist=twist1)
        expected_link_state2 = LinkState(link_name=link_name2,
                                         pose=pose2,
                                         twist=twist2)

        res = GetAllLinkStatesResponse()
        res.success = True
        res.status_message = ''
        res.link_states = [expected_link_state1.to_ros(),
                           expected_link_state2.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_link_states_mock.return_value = res

        tracker = GetLinkStateTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        link_state1 = tracker.get_link_state(link_name1)
        link_state2 = tracker.get_link_state(link_name2)

        assert link_state1 == expected_link_state1
        assert link_state2 == expected_link_state2

        # Check returned link state is a copy
        assert tracker._link_map[link_name1] == link_state1
        assert tracker._link_map[link_name1] is not link_state1
        assert tracker._link_map[link_name2] == link_state2
        assert tracker._link_map[link_name2] is not link_state2
        self.get_link_states_mock.assert_not_called()

    def test_get_link_state_blocking(self, service_proxy_wrapper_mock):
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        expected_link_state = LinkState(link_name=link_name,
                                        pose=pose,
                                        twist=twist)

        res = GetLinkStatesResponse()
        res.status = [True]
        res.success = True
        res.link_states = [expected_link_state.to_ros()]
        
        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_link_states_mock.return_value = res
        
        tracker = GetLinkStateTracker(is_singleton=False)
        link_state = tracker.get_link_state(link_name, blocking=True)
        self.get_link_states_mock.assert_called_once_with([link_name], [''])
        assert link_state == expected_link_state

    def test_get_link_state_missing_in_dict(self, service_proxy_wrapper_mock):
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        expected_link_state = LinkState(link_name=link_name,
                                        pose=pose,
                                        twist=twist)

        res = GetLinkStatesResponse()
        res.status = [True]
        res.success = True
        res.link_states = [expected_link_state.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_link_states_mock.return_value = res

        tracker = GetLinkStateTracker(is_singleton=False)
        link_state = tracker.get_link_state(link_name)
        self.get_link_states_mock.assert_called_once_with([link_name], [''])
        assert link_state == expected_link_state

    def test_get_link_state_missing_and_failed_to_retrieve(self, service_proxy_wrapper_mock):
        link_name = myself()

        res = GetLinkStatesResponse()
        res.status = [False]
        res.success = False
        res.link_states = [None]
        res.messages = ["Failed"]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_link_states_mock.return_value = res

        tracker = GetLinkStateTracker(is_singleton=False)
        with self.assertRaises(DeepSimException):
            tracker.get_link_state(link_name)

        res.status = [False]
        res.success = True

        with self.assertRaises(DeepSimException):
            tracker.get_link_state(link_name)

        res.status = [True]
        res.success = False

        with self.assertRaises(DeepSimException):
            tracker.get_link_state(link_name)

        self.get_link_states_mock.has_calls(
            call([link_name], ['']),
            call([link_name], ['']),
            call([link_name], [''])
        )

    def test_get_link_state_with_reference_frame(self, service_proxy_wrapper_mock):
        reference_frame = myself() + "_reference_frame"

        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))

        expected_link_state = LinkState(link_name=link_name,
                                        pose=pose,
                                        twist=twist)

        res = GetAllLinkStatesResponse()
        res.success = True
        res.status_message = ''
        res.link_states = [expected_link_state.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_link_states_mock.return_value = res

        tracker = GetLinkStateTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        expected_link_state_w_ref = LinkState(link_name=link_name,
                                              pose=pose,
                                              twist=twist,
                                              reference_frame=reference_frame)

        res = GetLinkStatesResponse()
        res.status = [True]
        res.success = True
        res.link_states = [expected_link_state_w_ref.to_ros()]
        self.get_link_states_mock.return_value = res

        link_state = tracker.get_link_state(link_name)

        assert link_state == expected_link_state
        # Check returned link state is a copy
        assert tracker._link_map[link_name] == link_state
        assert tracker._link_map[link_name] is not link_state

        link_state_w_ref = tracker.get_link_state(link_name, reference_frame=reference_frame)
        assert link_state_w_ref == expected_link_state_w_ref
        self.get_link_states_mock.assert_called_once_with([link_name], [reference_frame])

    def test_get_link_states(self, service_proxy_wrapper_mock):
        link_name1 = myself() + "1"
        link_name2 = myself() + "2"
        pose1 = Pose(position=Vector3(1.0, 2.0, 3.0),
                     orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        pose2 = Pose(position=Vector3(3.0, 4.0, 5.0),
                     orientation=Quaternion(4.0, 5.0, 6.0, 7.0))
        twist1 = Twist(linear=Vector3(2.0, 3.0, 4.0),
                       angular=Vector3(3.0, 4.0, 5.0))
        twist2 = Twist(linear=Vector3(3.0, 4.0, 5.0),
                       angular=Vector3(4.0, 5.0, 6.0))

        link_state1 = LinkState(link_name=link_name1,
                                pose=pose1,
                                twist=twist1)
        link_state2 = LinkState(link_name=link_name2,
                                pose=pose2,
                                twist=twist2)
        res = GetAllLinkStatesResponse()
        res.success = True
        res.status_message = ''
        res.link_states = [link_state1.to_ros(), link_state2.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_link_states_mock.return_value = res

        tracker = GetLinkStateTracker(is_singleton=False)

        tracker.on_update_tracker(0.1, None)

        link_states = tracker.get_link_states([link_name1, link_name2])

        key1 = (link_name1, '')
        key2 = (link_name2, '')
        expected_link_states = {key1: LinkState(link_name=link_name1,
                                                pose=pose1,
                                                twist=twist1),
                                key2: LinkState(link_name=link_name2,
                                                pose=pose2,
                                                twist=twist2)}

        assert link_states == expected_link_states

        self.get_link_states_mock.assert_not_called()

    def test_get_link_states_blocking(self, service_proxy_wrapper_mock):
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        expected_link_state = LinkState(link_name=link_name,
                                        pose=pose,
                                        twist=twist)
        key = (link_name, '')
        expected_return = {key: expected_link_state}

        res = GetLinkStatesResponse()
        res.status = [True]
        res.success = True
        res.link_states = [expected_link_state.to_ros()]
        self.get_link_states_mock.return_value = res
        service_proxy_wrapper_mock.side_effect = self.get_service_mock

        tracker = GetLinkStateTracker(is_singleton=False)
        link_state = tracker.get_link_states([link_name], blocking=True)
        self.get_link_states_mock.assert_called_once_with([link_name], [''])
        assert link_state == expected_return

    def test_get_link_states_missing_in_dict(self, service_proxy_wrapper_mock):
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        expected_link_state = LinkState(link_name=link_name,
                                        pose=pose,
                                        twist=twist)
        key = (link_name, '')
        expected_return = {key: expected_link_state}

        res = GetLinkStatesResponse()
        res.status = [True]
        res.success = True
        res.link_states = [expected_link_state.to_ros()]
        self.get_link_states_mock.return_value = res
        service_proxy_wrapper_mock.side_effect = self.get_service_mock

        tracker = GetLinkStateTracker(is_singleton=False)
        link_state = tracker.get_link_states([link_name])
        self.get_link_states_mock.assert_called_once_with([link_name], [''])
        assert link_state == expected_return

    def test_get_link_states_missing_and_failed_to_retrieve(self, service_proxy_wrapper_mock):
        link_name = myself()

        res = GetLinkStatesResponse()
        res.status = [False]
        res.success = False
        res.link_states = [None]
        res.messages = ["Failed"]

        self.get_link_states_mock.return_value = res
        service_proxy_wrapper_mock.side_effect = self.get_service_mock

        tracker = GetLinkStateTracker(is_singleton=False)
        with self.assertRaises(DeepSimException):
            tracker.get_link_states([link_name])

        res.status = [False]
        res.success = True

        assert tracker.get_link_states([link_name]) == {(link_name, ''): None}

        res.status = [True]
        res.success = False

        with self.assertRaises(DeepSimException):
            tracker.get_link_states([link_name])

        self.get_link_states_mock.has_calls(
            call([link_name], ['']),
            call([link_name], ['']),
            call([link_name], [''])
        )

    def test_get_link_states_with_reference_frame(self, service_proxy_wrapper_mock):
        reference_frame = myself() + "_reference_frame"

        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))

        expected_link_state = LinkState(link_name=link_name,
                                        pose=pose,
                                        twist=twist)

        res = GetAllLinkStatesResponse()
        res.success = True
        res.status_message = ''
        res.link_states = [expected_link_state.to_ros()]

        service_proxy_wrapper_mock.side_effect = self.get_service_mock
        self.get_all_link_states_mock.return_value = res

        tracker = GetLinkStateTracker(is_singleton=False)
        tracker.on_update_tracker(0.1, None)

        expected_link_state_w_ref = LinkState(link_name=link_name,
                                              pose=pose,
                                              twist=twist,
                                              reference_frame=reference_frame)
        key = (link_name, reference_frame)
        expected_return = {key: expected_link_state_w_ref}

        res = GetLinkStatesResponse()
        res.status = [True]
        res.success = True
        res.link_states = [expected_link_state_w_ref.to_ros()]
        self.get_link_states_mock.return_value = res

        link_state = tracker.get_link_state(link_name)

        assert link_state == expected_link_state
        # Check returned link state is a copy
        assert tracker._link_map[link_name] == link_state
        assert tracker._link_map[link_name] is not link_state

        link_state_w_ref = tracker.get_link_states([link_name], reference_frames=[reference_frame])
        assert link_state_w_ref == expected_return
        self.get_link_states_mock.assert_called_once_with([link_name], [reference_frame])

    def test_get_link_states_unmatched_length(self, service_proxy_wrapper_mock):
        tracker = GetLinkStateTracker(is_singleton=False)

        link_name = myself()
        with self.assertRaises(ValueError):
            tracker.get_link_states([link_name], [])

    def test_set_link_state(self, service_proxy_wrapper_mock):
        tracker = GetLinkStateTracker(is_singleton=False)
        link_name = myself()
        expected_link_state = LinkState(link_name=link_name)
        tracker.set_link_state(expected_link_state)
        assert expected_link_state == tracker.get_link_state(name=link_name)
