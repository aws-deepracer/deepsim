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
# Custom message for an array of Visuals
string[] link_name
string[] visual_name
std_msgs/ColorRGBA[] ambient
std_msgs/ColorRGBA[] diffuse
std_msgs/ColorRGBA[] specular
std_msgs/ColorRGBA[] emissive
float64[] transparency
bool[] visible
uint16[] geometry_type
string[] mesh_geom_filename
geometry_msgs/Vector3[] mesh_geom_scale
geometry_msgs/Pose[] pose