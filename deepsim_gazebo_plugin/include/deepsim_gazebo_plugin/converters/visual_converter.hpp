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

#ifndef DEEPSIM_GAZEBO_PLUGIN__VISUAL_CONVERTER_HPP_
#define DEEPSIM_GAZEBO_PLUGIN__VISUAL_CONVERTER_HPP_

#include <std_msgs/ColorRGBA.h>
#include <gazebo/msgs/msgs.hh>
#include "deepsim_msgs/Visual.h"

#include <string>

namespace deepsim_gazebo_plugin
{
class VisualConverter
{
public:
    /// \brief Specialized conversion from a Gazebo Color msg to std_msgs::ColorRGBA
    /// \param[in] msg gazebo color message to convert.
    /// \return A standard ColorRGBA
    static std_msgs::ColorRGBA Convert2StdColorRGBA(const gazebo::msgs::Color & msg);

    /// \brief Specialized setter from std_msgs::ColorRGBA to a Gazebo Color msg
    /// \param[in] msg Gazebo Color msg
    /// \param[in] in standard color message to convert.
    static void SetGazeboColor(gazebo::msgs::Color *_msg, const std_msgs::ColorRGBA & in);

    /// \brief Specialized conversion from gazebo::msgs::Visual to deepsim_msgs::Visual
    /// \param[in] link_name link name.
    /// \param[in] visual_name visual name.
    /// \param[in] msg gazebo Visual message to convert.
    /// \return A deepsim Visual message
    static deepsim_msgs::Visual Convert2DeepsimVisual(
      const std::string& link_name,
      const std::string& visual_name,
      const gazebo::msgs::Visual & msg);

    /// \brief Get a default Black Opaque color in Standrad Color RGBA
    /// \return A Standrad Color RGBA in Black Opaque
    static std_msgs::ColorRGBA StandardBlackOpaque();
};
}  // namespace deepsim_gazebo_plugin
#endif  // DEEPSIM_GAZEBO_PLUGIN__VISUAL_CONVERTER_HPP_
