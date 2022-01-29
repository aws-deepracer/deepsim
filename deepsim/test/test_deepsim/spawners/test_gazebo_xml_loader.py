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
"""this module test gazebo xml loader"""
from unittest import mock, TestCase
from unittest.mock import patch, MagicMock, call, ANY, mock_open

from deepsim.spawners.gazebo_xml_loader import GazeboXmlLoader


@patch("builtins.open", new_callable=mock_open, read_data="data")
@patch("deepsim.spawners.gazebo_xml_loader.xacro")
class GazeboXmlLoaderTest(TestCase):
    def test_sdf_file_extension(self, xacro_mock, mock_open):
        """test file with sdf as extension

        Args:
            xacro_mock (Mock): xacro mock
            mock_open (Mock): open mock
        """
        result = GazeboXmlLoader.parse(file_path="blabla.sdf")
        mock_open.assert_called_once()
        self.assertEqual(result, "data")

    def test_urdf_file_extension(self, xacro_mock, mock_open):
        """test file with urdf as extension

        Args:
            xacro_mock (Mock): xacro mock
            mock_open (Mock): open mock
        """
        result = GazeboXmlLoader.parse(file_path="blabla.urdf")
        mock_open.assert_called_once()
        self.assertEqual(result, "data")

    def test_xacro_file_extension(self, xacro_mock, mock_open):
        """test file with xacro as extension

        Args:
            xacro_mock (Mock): xacro mock
            mock_open (Mock): open mock
        """
        xacro_mock.process_file.return_value.toxml.return_value = "data"
        result = GazeboXmlLoader.parse(file_path="blabla.xacro", key="value")
        xacro_mock.process_file.assert_called_once_with(input_file_name="blabla.xacro", mappings={"key": "value"})
        self.assertEqual(result, "data")

    def test_invalid_file_extension_exception(self, xacro_mock, mock_open):
        """test file with invalid extension .blabla

        Args:
            xacro_mock (Mock): xacro mock
            mock_open (Mock): open mock
        """
        with self.assertRaises(ValueError) as context:
            GazeboXmlLoader.parse(file_path="blabla.blabla")

    def test_sdf_open_exception(self, xacro_mock, mock_open):
        """test sdf open exception

        Args:
            xacro_mock (Mock): xacro mock
            mock_open (Mock): open mock
        """
        with self.assertRaises(Exception) as context:
            mock_open.side_effect = Exception("sdf open fault")
            GazeboXmlLoader.parse(file_path="blabla.sdf")

    def test_urdf_open_exception(self, xacro_mock, mock_open):
        """test urdf open exception

        Args:
            xacro_mock (Mock): xacro mock
            mock_open (Mock): open mock
        """
        with self.assertRaises(Exception) as context:
            mock_open.side_effect = Exception("urdf open fault")
            GazeboXmlLoader.parse(file_path="blabla.urdf")

    def test_xacro_process_exception(self, xacro_mock, mock_open):
        """test xacro process exception

        Args:
            xacro_mock (Mock): xacro mock
            mock_open (Mock): open mock
        """
        with self.assertRaises(Exception) as context:
            xacro_mock.process_file.side_effect = Exception("xacro process fault")
            GazeboXmlLoader.parse(file_path="blabla.xacro")
