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

#ifndef DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_VISUAL_SERVICES_HPP_
#define DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_VISUAL_SERVICES_HPP_

#include <memory>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#define protected public
#include <gazebo/physics/physics.hh>
#undef protected

#include "deepsim_msgs/Visual.h"
#include "deepsim_msgs/GetVisual.h"
#include "deepsim_msgs/GetVisuals.h"
#include "deepsim_msgs/GetAllVisuals.h"
#include "deepsim_msgs/GetVisualNames.h"
#include "deepsim_msgs/SetVisualMaterial.h"
#include "deepsim_msgs/SetVisualMaterials.h"
#include "deepsim_msgs/SetVisualTransparency.h"
#include "deepsim_msgs/SetVisualTransparencies.h"
#include "deepsim_msgs/SetVisualVisible.h"
#include "deepsim_msgs/SetVisualVisibles.h"

#include "deepsim_gazebo_plugin/converters/visual_converter.hpp"
#include "deepsim_gazebo_plugin/deepsim_gazebo_visual_publisher.hpp"

namespace deepsim_gazebo_plugin
{

class DeepsimGazeboVisualServicesPrivate;

class DeepsimGazeboVisualServices
{
public:
  /// Constructor
  explicit DeepsimGazeboVisualServices(
    std::mutex & _lock,
    ros::CallbackQueue & ros_callback_queue_);

  /// Destructor
  virtual ~DeepsimGazeboVisualServices();

  /// \brief Create visual related services.
  /// \param[in] _gz_node gazebo::transport::NodePtr
  /// The gazebo node pointer for gazebo transport.
  /// \param[in] _ros_node gazebo_ros::Node::SharedPtr
  /// The ros node pointer for logging and beyond.
  /// \param[in] _world gazebo_ros::Node::SharedPtr
  /// The gazebo world pointer to fetch additional entities.
  void CreateServices(
    const gazebo::transport::NodePtr _gz_node,
    const boost::shared_ptr<ros::NodeHandle> _ros_node,
    const gazebo::physics::WorldPtr _world);

private:
  std::unique_ptr<DeepsimGazeboVisualServicesPrivate> impl_;
};
}  // namespace deepsim_gazebo_plugin
#endif  // DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_VISUAL_SERVICES_HPP_
