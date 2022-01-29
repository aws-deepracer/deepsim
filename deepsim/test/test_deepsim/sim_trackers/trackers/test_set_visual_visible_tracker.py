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

from deepsim.sim_trackers.trackers.set_visual_visible_tracker import SetVisualVisibleTracker
from deepsim.gazebo.constants import GazeboServiceName

from deepsim_msgs.srv import (
    SetVisualVisibles, SetVisualVisiblesRequest, SetVisualVisiblesResponse,
    SetVisualVisible, SetVisualVisibleResponse
)

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


@patch("deepsim.sim_trackers.trackers.set_visual_visible_tracker.GetVisualTracker")
@patch("deepsim.sim_trackers.trackers.set_visual_visible_tracker.ServiceProxyWrapper")
class SetVisualVisibleStateTrackerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        _ = SetVisualVisibleTracker(is_singleton=False)
        service_proxy_wrapper_mock.assert_called_once_with(GazeboServiceName.SET_VISUAL_VISIBLES,
                                                           SetVisualVisibles)

    def test_set_visual_visible(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualVisibleTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        visible = False

        tracker.set_visual_visible(link_name=link_name,
                                   visual_name=visual_name,
                                   visible=visible)
        key = (link_name, visual_name)
        assert tracker._visual_name_map[key] == visual_name
        assert tracker._link_name_map[key] == link_name
        assert tracker._visible_map[key] == visible

    def test_set_visual_visible_blocking(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualVisibleTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        visible = False

        tracker.set_visual_visible(link_name=link_name,
                                   visual_name=visual_name,
                                   visible=visible)

        res = SetVisualVisiblesResponse()
        res.success = True
        res.status.append(True)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetVisualVisibleResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        msg = tracker.set_visual_visible(link_name=link_name,
                                         visual_name=visual_name,
                                         visible=visible,
                                         blocking=True)

        req = SetVisualVisiblesRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.visibles = [visible]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        get_visual_tracker_mock_obj = get_visual_tracker_mock.get_instance.return_value
        get_visual_tracker_mock_obj.set_visible.assert_called_once_with(link_name=link_name,
                                                                        visual_name=visual_name,
                                                                        visible=visible)

    def test_set_visual_visible_blocking_override(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualVisibleTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        visible = False

        res = SetVisualVisiblesResponse()
        res.success = True
        res.status.append(True)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetVisualVisibleResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        _ = tracker.set_visual_visible(link_name=link_name,
                                       visual_name=visual_name,
                                       visible=visible)

        # Confirm new model_state is persisted to map
        key = (link_name, visual_name)
        assert tracker._visual_name_map[key] == visual_name
        assert tracker._link_name_map[key] == link_name
        assert tracker._visible_map[key] == visible

        msg = tracker.set_visual_visible(link_name=link_name,
                                         visual_name=visual_name,
                                         visible=visible,
                                         blocking=True)

        # Confirm the service is called with new model state.
        req = SetVisualVisiblesRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.visibles = [visible]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        # Confirm the new model state is removed from the map.
        key = (link_name, visual_name)
        assert key not in tracker._visual_name_map
        assert key not in tracker._link_name_map
        assert key not in tracker._visible_map
        get_visual_tracker_mock_obj = get_visual_tracker_mock.get_instance.return_value
        get_visual_tracker_mock_obj.set_visible.assert_called_once_with(link_name=link_name,
                                                                        visual_name=visual_name,
                                                                        visible=visible)

    def test_set_visual_visible_blocking_failed(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualVisibleTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        visible = False

        tracker.set_visual_visible(link_name=link_name,
                                   visual_name=visual_name,
                                   visible=visible)

        res = SetVisualVisiblesResponse()
        res.success = False
        res.status.append(False)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetVisualVisibleResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        msg = tracker.set_visual_visible(link_name=link_name,
                                         visual_name=visual_name,
                                         visible=visible,
                                         blocking=True)

        req = SetVisualVisiblesRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.visibles = [visible]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        get_visual_tracker_mock.get_instance.return_value.set_visible.assert_not_called()

    def test_set_visual_visibles(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualVisibleTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        visible = False

        tracker.set_visual_visibles(link_names=[link_name],
                                    visual_names=[visual_name],
                                    visibles=[visible])
        key = (link_name, visual_name)
        assert tracker._visual_name_map[key] == visual_name
        assert tracker._link_name_map[key] == link_name
        assert tracker._visible_map[key] == visible

    def test_set_visual_visibles_blocking(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualVisibleTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        visible = False

        tracker.set_visual_visibles(link_names=[link_name],
                                    visual_names=[visual_name],
                                    visibles=[visible])

        expected_msg = SetVisualVisiblesResponse()
        expected_msg.success = True
        expected_msg.status.append(True)
        expected_msg.messages.append("")

        service_proxy_wrapper_mock.return_value.return_value = expected_msg
        msg = tracker.set_visual_visibles(link_names=[link_name],
                                          visual_names=[visual_name],
                                          visibles=[visible],
                                          blocking=True)

        req = SetVisualVisiblesRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.visibles = [visible]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        get_visual_tracker_mock_obj = get_visual_tracker_mock.get_instance.return_value
        get_visual_tracker_mock_obj.set_visible.assert_called_once_with(link_name=link_name,
                                                                        visual_name=visual_name,
                                                                        visible=visible)

    def test_set_visual_visibles_blocking_failed(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualVisibleTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        visible = False

        tracker.set_visual_visibles(link_names=[link_name],
                                    visual_names=[visual_name],
                                    visibles=[visible])

        expected_msg = SetVisualVisiblesResponse()
        expected_msg.success = False
        expected_msg.status.append(False)
        expected_msg.messages.append("")

        service_proxy_wrapper_mock.return_value.return_value = expected_msg
        msg = tracker.set_visual_visibles(link_names=[link_name],
                                          visual_names=[visual_name],
                                          visibles=[visible],
                                          blocking=True)

        req = SetVisualVisiblesRequest()
        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.visibles = [visible]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)
        assert msg == expected_msg
        get_visual_tracker_mock.get_instance.return_value.set_visible.assert_not_called()

    def test_on_update_tracker(self, service_proxy_wrapper_mock, get_visual_tracker_mock):
        tracker = SetVisualVisibleTracker(is_singleton=False)
        visual_name = myself() + "_visual"
        link_name = myself()
        visible = False

        res = SetVisualVisiblesResponse()
        res.success = True
        res.status.append(True)
        res.messages.append("Done")
        res.status_message = "Done"

        expected_msg = SetVisualVisibleResponse()
        expected_msg.success = res.success and res.status[0]
        expected_msg.status_message = res.messages[0] if res.success else res.status_message

        service_proxy_wrapper_mock.return_value.return_value = res
        _ = tracker.set_visual_visible(link_name=link_name,
                                       visual_name=visual_name,
                                       visible=visible)

        # Confirm new model_state is persisted to map
        key = (link_name, visual_name)
        assert tracker._visual_name_map[key] == visual_name
        assert tracker._link_name_map[key] == link_name
        assert tracker._visible_map[key] == visible

        tracker.on_update_tracker(MagicMock(), MagicMock())
        # Confirm the service is called with new model state.
        req = SetVisualVisiblesRequest()

        req.visual_names = [visual_name]
        req.link_names = [link_name]
        req.visibles = [visible]
        req.block = True

        service_proxy_wrapper_mock.return_value.assert_called_once_with(req)

        key = (link_name, visual_name)
        assert key not in tracker._visual_name_map
        assert key not in tracker._link_name_map
        assert key not in tracker._visible_map
