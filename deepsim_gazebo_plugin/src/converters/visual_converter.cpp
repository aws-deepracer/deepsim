///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#include "deepsim_gazebo_plugin/converters/visual_converter.hpp"
#include "deepsim_gazebo_plugin/converters/geometry_converter.hpp"



namespace deepsim_gazebo_plugin
{
/// \brief Specialized conversion from a Gazebo Color msg to std_msgs::ColorRGBA
/// \param[in] msg gazebo color message to convert.
/// \return A standard ColorRGBA
std_msgs::ColorRGBA VisualConverter::Convert2StdColorRGBA(const gazebo::msgs::Color & in)
{
  std_msgs::ColorRGBA std_c;
  std_c.r = in.r();
  std_c.g = in.g();
  std_c.b = in.b();
  std_c.a = in.a();
  return std_c;
}

    /// \brief Specialized setter from std_msgs::ColorRGBA to a Gazebo Color msg
    /// \param[in] msg Gazebo Color msg
    /// \param[in] in standard color message to convert.
void VisualConverter::SetGazeboColor(gazebo::msgs::Color *_msg, const std_msgs::ColorRGBA & in)
{
  _msg->set_r(in.r);
  _msg->set_g(in.g);
  _msg->set_b(in.b);
  _msg->set_a(in.a);
}

/// \brief Get a default Black Opaque color in standard Color RGBA
/// \return A standard Color RGBA in Black Opaque
std_msgs::ColorRGBA VisualConverter::StandardBlackOpaque()
{
  std_msgs::ColorRGBA std_c;
  std_c.r = 0.0f;
  std_c.g = 0.0f;
  std_c.b = 0.0f;
  std_c.a = 1.0f;
  return std_c;
}

deepsim_msgs::Visual VisualConverter::Convert2DeepsimVisual(
  const std::string& link_name,
  const std::string& visual_name,
  const gazebo::msgs::Visual & visual_msg)
{
  deepsim_msgs::Visual visual;

  visual.link_name = link_name;
  visual.visual_name = visual_name;

  const gazebo::msgs::Material &material_msg = visual_msg.has_material() ?
    visual_msg.material() : gazebo::msgs::Material();
  // std_msgs/ColorRGBA ambient => default black opaque
  visual.ambient = material_msg.has_ambient() ?
    VisualConverter::Convert2StdColorRGBA(material_msg.ambient()) :
    VisualConverter::StandardBlackOpaque();
  // std_msgs/ColorRGBA diffuse => default black opaque
  visual.diffuse = material_msg.has_diffuse() ?
    VisualConverter::Convert2StdColorRGBA(material_msg.diffuse()) :
    VisualConverter::StandardBlackOpaque();
  // std_msgs/ColorRGBA specular => default black opaque
  visual.specular = material_msg.has_specular() ?
    VisualConverter::Convert2StdColorRGBA(material_msg.specular()) :
    VisualConverter::StandardBlackOpaque();
  // std_msgs/ColorRGBA emissive => default black opaque
  visual.emissive =
    material_msg.has_emissive() ?
    VisualConverter::Convert2StdColorRGBA(material_msg.emissive()) :
    VisualConverter::StandardBlackOpaque();
  // float64 transparency        => default 0.0
  visual.transparency = visual_msg.has_transparency()? visual_msg.transparency(): 0.0;
  // bool visible                => default false
  visual.visible = visual_msg.has_visible()? visual_msg.visible(): false;
  // uint16 geometry_type        => default to Type Empty
  const gazebo::msgs::Geometry &geometry_msg =
    visual_msg.has_geometry() ? visual_msg.geometry() : gazebo::msgs::Geometry();
  visual.geometry_type = geometry_msg.has_type() ?
    (unsigned int) geometry_msg.type() :
    gazebo::msgs::Geometry_Type_EMPTY;
  // string mesh_geom_filename   => default to empty string
  const gazebo::msgs::MeshGeom &geom_mesh_msg =
    geometry_msg.has_mesh() ? geometry_msg.mesh() : gazebo::msgs::MeshGeom();
  visual.mesh_geom_filename = geom_mesh_msg.has_filename() ? geom_mesh_msg.filename() : "";
  // geometry_msgs/Vector3 mesh_geom_scale => default to all ones
  visual.mesh_geom_scale = geom_mesh_msg.has_scale() ?
    deepsim_gazebo_plugin::GeometryConverter::Convert2GeoVector3(geom_mesh_msg.scale()):
    deepsim_gazebo_plugin::GeometryConverter::GeoVector3Ones();
  // geometry_msgs/Pose pose  => default to empty pose.
  visual.pose = visual_msg.has_pose() ?
    deepsim_gazebo_plugin::GeometryConverter::Convert2GeoPose(visual_msg.pose()) :
    geometry_msgs::Pose();
  return visual;
}
}  // namespace deepsim_gazebo_plugin
