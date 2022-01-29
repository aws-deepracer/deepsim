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

#include "deepsim_gazebo_plugin/converters/geometry_converter.hpp"


namespace deepsim_gazebo_plugin
{

/// \brief Specialized conversion from a ROS vector message to an Ignition Math vector.
/// \param[in] msg ROS message to convert.
/// \return An Ignition Math vector.
ignition::math::Vector3d GeometryConverter::Convert2MathVector3d(const geometry_msgs::Vector3 & msg)
{
  ignition::math::Vector3d vec;
  vec.X(msg.x);
  vec.Y(msg.y);
  vec.Z(msg.z);
  return vec;
}


/// \brief Specialized conversion from a ROS point message to a ROS vector message.
/// \param[in] in ROS message to convert.
/// \return geometry_msgs::Vector3
geometry_msgs::Vector3 GeometryConverter::Convert2GeoVector3(const geometry_msgs::Point & in)
{
  geometry_msgs::Vector3 msg;
  msg.x = in.x;
  msg.y = in.y;
  msg.z = in.z;
  return msg;
}

/// \brief Specialized conversion from a ROS point message to an Ignition math vector.
/// \param[in] in ROS message to convert.
/// \return An Ignition Math vector.
ignition::math::Vector3d GeometryConverter::Convert2MathVector3d(const geometry_msgs::Point & in)
{
  ignition::math::Vector3d out;
  out.X(in.x);
  out.Y(in.y);
  out.Z(in.z);
  return out;
}

/// \brief Specialized conversion from an Ignition Math vector to a ROS message.
/// \param[in] vec Ignition vector to convert.
/// \return ROS geometry vector message
geometry_msgs::Vector3 GeometryConverter::Convert2GeoVector3(
    const ignition::math::Vector3d & vec)
{
  geometry_msgs::Vector3 msg;
  msg.x = vec.X();
  msg.y = vec.Y();
  msg.z = vec.Z();
  return msg;
}

/// \brief Specialized conversion from an Ignition Math vector to a ROS message.
/// \param[in] vec Ignition vector to convert.
/// \return ROS geometry point message
geometry_msgs::Point GeometryConverter::Convert2GeoPoint(
    const ignition::math::Vector3d & vec)
{
  geometry_msgs::Point msg;
  msg.x = vec.X();
  msg.y = vec.Y();
  msg.z = vec.Z();
  return msg;
}

/// \brief Specialized conversion from an Ignition Math Quaternion to a ROS message.
/// \param[in] in Ignition Quaternion to convert.
/// \return ROS geometry quaternion message
geometry_msgs::Quaternion GeometryConverter::Convert2GeoQuaternion(
    const ignition::math::Quaterniond & in)
{
  geometry_msgs::Quaternion msg;
  msg.x = in.X();
  msg.y = in.Y();
  msg.z = in.Z();
  msg.w = in.W();
  return msg;
}

/// \brief Specialized conversion from an Ignition Math Pose3d to a ROS geometry transform message.
/// \param[in] in Ignition Pose3d to convert.
/// \return ROS geometry transform message
geometry_msgs::Transform GeometryConverter::Convert2GeoTransform(
    const ignition::math::Pose3d & in)
{
  geometry_msgs::Transform msg;
  msg.translation = Convert2GeoVector3(in.Pos());
  msg.rotation = Convert2GeoQuaternion(in.Rot());
  return msg;
}

/// \brief Specialized conversion from an Ignition Math Pose3d to a ROS geometry pose message.
/// \param[in] in Ignition Pose3d to convert.
/// \return ROS geometry pose message
geometry_msgs::Pose GeometryConverter::Convert2GeoPose(
    const ignition::math::Pose3d & in)
{
  geometry_msgs::Pose msg;
  msg.position = GeometryConverter::Convert2GeoPoint(in.Pos());
  msg.orientation = GeometryConverter::Convert2GeoQuaternion(in.Rot());
  return msg;
}


/// \brief Specialized conversion from a ROS quaternion message to ignition quaternion
/// \param[in] in Input quaternion message
/// \return Ignition math quaternion with same values as the input message
ignition::math::Quaterniond GeometryConverter::Convert2MathQuaterniond(
  const geometry_msgs::Quaternion & in)
{
  return ignition::math::Quaterniond(in.w, in.x, in.y, in.z);
}


/// \brief Specialized conversion from a ROS geometry transform message to an Ignition math pose3d.
/// \param[in] in ROS message to convert.
/// \return A Ignition Math pose3d.
ignition::math::Pose3d GeometryConverter::Convert2MathPose3d(
    const geometry_msgs::Transform & in)
{
  ignition::math::Pose3d msg;
  msg.Pos() = GeometryConverter::Convert2MathVector3d(in.translation);
  msg.Rot() = GeometryConverter::Convert2MathQuaterniond(in.rotation);
  return msg;
}


/// \brief Specialized conversion from a ROS pose message to a ROS geometry transform message.
/// \param[in] in ROS pose message to convert.
/// \return A ROS geometry transform message.
geometry_msgs::Transform GeometryConverter::Convert2GeoTransform(
    const geometry_msgs::Pose & in)
{
  geometry_msgs::Transform msg;
  msg.translation = Convert2GeoVector3(in.position);
  msg.rotation = in.orientation;
  return msg;
}

/// \brief Specialized conversion from a ROS pose message to an Ignition Math pose.
/// \param[in] in ROS pose message to convert.
/// \return Ignition Math pose.
ignition::math::Pose3d GeometryConverter::Convert2MathPose3d(
    const geometry_msgs::Pose & in)
{
  return
  {
    Convert2MathVector3d(in.position),
    Convert2MathQuaterniond(in.orientation)
  };
}

/// \brief Specialized conversion from a ROS pose message to an Ignition Math pose.
/// \param[in] in ROS pose message to convert.
/// \return ROS geometry pose message
geometry_msgs::Pose GeometryConverter::Convert2GeoPose(const gazebo::msgs::Pose & in)
{
  geometry_msgs::Pose msg;
  msg.position = Convert2GeoPoint(in.position());
  msg.orientation = Convert2GeoQuaternion(in.orientation());
  return msg;
}

/// \brief Specialized conversion from a gazebo Vector3d message to ROS geometry_msgs::Vector3.
/// \param[in] in the gazebo msgs::Vector3d to convert.
/// \return ROS geometry_msgs::Vector3.
geometry_msgs::Vector3 GeometryConverter::Convert2GeoVector3(const gazebo::msgs::Vector3d & in)
{
  geometry_msgs::Vector3 msg;
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  return msg;
}

/// \brief Specialized conversion from a gazebo Quaternion message to an ROS geometry_msgs::Quaternion.
/// \param[in] in gazebo::msgs::Quaternion message to convert.
/// \return ROS geometry_msgs::Quaternion.
geometry_msgs::Quaternion GeometryConverter::Convert2GeoQuaternion(const gazebo::msgs::Quaternion & in)
{
  geometry_msgs::Quaternion msg;
  msg.x = in.x();
  msg.y = in.y();
  msg.z = in.z();
  msg.w = in.w();
  return msg;
}

/// \brief Specialized conversion from a gazebo vector3d message to an ROS geometry_msgs::Point message.
/// \param[in] in gazebo::msgs::Vector3d gazebo message to convert
/// \return ROS geometry_msgs::Point message
geometry_msgs::Point GeometryConverter::Convert2GeoPoint(const gazebo::msgs::Vector3d & in)
{
  geometry_msgs::Point point;
  point.x = in.x();
  point.y = in.y();
  point.z = in.z();
  return point;
}

/// \brief Get a geometry_msgs::Vector3 of all ones
/// \return geometry_msgs::Vector3 of all ones
geometry_msgs::Vector3 GeometryConverter::GeoVector3Ones()
{
  geometry_msgs::Vector3 vec;
  vec.x = 1.0;
  vec.y = 1.0;
  vec.z = 1.0;
  return vec;
}
}  // namespace deepsim_gazebo_plugin
