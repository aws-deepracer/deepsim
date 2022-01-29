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
"""A class for Gazebo XML loader."""
import os
import xacro


class GazeboXmlLoader(object):
    """
    GazeboXmlLoader to parse sdf, urdf, and xacro file
    """
    @staticmethod
    def parse(file_path: str, **kwargs) -> str:
        """
        Parse sdf, urdf, or xacro file

        Args:
            file_path (str): file path to parse
            **kwargs: Arbitrary keyword arguments

        Returns:
            str: string of processed file contents

        Raises:
            Exception: GazeboXmlLoader parse file loading or non-recognized type
            exception
        """
        _, file_extension = os.path.splitext(file_path)
        if file_extension in ['.sdf', '.urdf']:
            with open(file_path, "r") as file_pointer:
                xml = file_pointer.read()
            return xml
        if file_extension == '.xacro':
            xacro_xml = xacro.process_file(input_file_name=file_path,
                                           mappings=kwargs).toxml()
            return xacro_xml
        raise ValueError("[GazeboXmlLoader]: file type {} not recognizable".format(file_extension))
