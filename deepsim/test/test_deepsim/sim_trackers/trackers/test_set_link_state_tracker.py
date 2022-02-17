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

from deepsim.sim_trackers.trackers.set_link_state_tracker import SetLinkStateTracker
from deepsim.gazebo.constants import GazeboServiceName
from deepsim.core.pose import Pose
from deepsim.core.twist import Twist
from deepsim.core.vector3 import Vector3
from deepsim.core.quaternion import Quaternion
from deepsim.core.link_state import LinkState

from deepsim_msgs.srv import SetLinkStates, SetLinkStatesResponse
from gazebo_msgs.srv import SetLinkState, SetLinkStateResponse

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.sim_trackers.trackers.set_link_state_tracker.GetLinkStateTracker")
@patch("deepsim.sim_trackers.trackers.set_link_state_tracker.ServiceProxyWrapper")
class SetLinkStateTrackerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, service_proxy_wrapper_mock, get_link_state_tracker_mock):
        _ = SetLinkStateTracker(is_singleton=False)
        service_proxy_wrapper_mock.assert_called_once_with(GazeboServiceName.SET_LINK_STATES,
                                                           SetLinkStates)

    def test_set_link_state(self, service_proxy_wrapper_mock, get_link_state_tracker_mock):
        tracker = SetLinkStateTracker(is_singleton=False)
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist)

        tracker.set_link_state(link_state)
        assert tracker._link_state_map[link_state.link_name] == link_state.to_ros()

    def test_set_link_state_blocking(self, service_proxy_wrapper_mock, get_link_state_tracker_mock):
        tracker = SetLinkStateTracker(is_singleton=False)
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist)
        res = SetLinkStatesResponse()
        res.success = True
        res.status.append(True)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetLinkStateResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        msg = tracker.set_link_state(link_state, blocking=True)

        service_proxy_wrapper_mock.return_value.assert_called_once_with([link_state.to_ros()])
        assert msg == expected_msg
        get_link_state_tracker_mock_obj = get_link_state_tracker_mock.get_instance.return_value
        get_link_state_tracker_mock_obj.set_link_state.assert_called_once_with(link_state=link_state)

    def test_set_link_state_blocking_override(self, service_proxy_wrapper_mock, get_link_state_tracker_mock):
        tracker = SetLinkStateTracker(is_singleton=False)
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist)
        res = SetLinkStatesResponse()
        res.success = True
        res.status.append(True)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetLinkStateResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        _ = tracker.set_link_state(link_state)

        # Confirm new link_state is persisted to map
        assert tracker._link_state_map[link_state.link_name] == link_state.to_ros()

        msg = tracker.set_link_state(link_state, blocking=True)
        # Confirm the service is called with new link state.
        service_proxy_wrapper_mock.return_value.assert_called_once_with([link_state.to_ros()])
        assert msg == expected_msg
        # Confirm the new link state is removed from the map.
        assert link_state.link_name not in tracker._link_state_map
        get_link_state_tracker_mock_obj = get_link_state_tracker_mock.get_instance.return_value
        get_link_state_tracker_mock_obj.set_link_state.assert_called_once_with(link_state=link_state)

    def test_set_link_state_blocking_failed(self, service_proxy_wrapper_mock, get_link_state_tracker_mock):
        tracker = SetLinkStateTracker(is_singleton=False)
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist)
        res = SetLinkStatesResponse()
        res.success = False
        res.status.append(False)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetLinkStateResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        msg = tracker.set_link_state(link_state, blocking=True)

        service_proxy_wrapper_mock.return_value.assert_called_once_with([link_state.to_ros()])
        assert msg == expected_msg
        get_link_state_tracker_mock.get_instance.return_value.set_link_state.assert_not_called()

    def test_set_link_states(self, service_proxy_wrapper_mock, get_link_state_tracker_mock):
        tracker = SetLinkStateTracker(is_singleton=False)
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist)

        tracker.set_link_states([link_state])
        assert tracker._link_state_map[link_state.link_name] == link_state.to_ros()

    def test_set_link_states_blocking(self, service_proxy_wrapper_mock, get_link_state_tracker_mock):
        tracker = SetLinkStateTracker(is_singleton=False)
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist)
        expected_msg = SetLinkStatesResponse()
        expected_msg.success = True
        expected_msg.status.append(True)
        expected_msg.messages.append("")

        service_proxy_wrapper_mock.return_value.return_value = expected_msg
        msg = tracker.set_link_states([link_state], blocking=True)

        service_proxy_wrapper_mock.return_value.assert_called_once_with([link_state.to_ros()])
        assert msg == expected_msg
        get_link_state_tracker_mock_obj = get_link_state_tracker_mock.get_instance.return_value
        get_link_state_tracker_mock_obj.set_link_state.assert_called_once_with(link_state=link_state)

    def test_set_link_states_blocking_failed(self, service_proxy_wrapper_mock, get_link_state_tracker_mock):
        tracker = SetLinkStateTracker(is_singleton=False)
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist)
        expected_msg = SetLinkStatesResponse()
        expected_msg.success = False
        expected_msg.status.append(False)
        expected_msg.messages.append("")

        service_proxy_wrapper_mock.return_value.return_value = expected_msg
        msg = tracker.set_link_states([link_state], blocking=True)

        service_proxy_wrapper_mock.return_value.assert_called_once_with([link_state.to_ros()])
        assert msg == expected_msg
        get_link_state_tracker_mock.get_instance.return_value.set_link_state.assert_not_called()

    def test_on_update_tracker(self, service_proxy_wrapper_mock, get_link_state_tracker_mock):
        tracker = SetLinkStateTracker(is_singleton=False)
        link_name = myself()
        pose = Pose(position=Vector3(1.0, 2.0, 3.0),
                    orientation=Quaternion(2.0, 3.0, 4.0, 5.0))
        twist = Twist(linear=Vector3(2.0, 3.0, 4.0),
                      angular=Vector3(3.0, 4.0, 5.0))
        link_state = LinkState(link_name=link_name,
                               pose=pose,
                               twist=twist)

        _ = tracker.set_link_state(link_state)

        # Confirm new link_state is persisted to map
        assert tracker._link_state_map[link_state.link_name] == link_state.to_ros()

        tracker.on_update_tracker(MagicMock(), MagicMock())
        # Confirm the service is called with new link state.
        service_proxy_wrapper_mock.return_value.assert_called_once_with([link_state.to_ros()])

        assert link_state.link_name not in tracker._link_state_map

