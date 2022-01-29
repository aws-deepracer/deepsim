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

#ifndef DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_SYSTEM_PLUGIN_HPP_
#define DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_SYSTEM_PLUGIN_HPP_

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_options.h>
#include <std_msgs/ColorRGBA.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// Gazebo
#define protected public
#include <gazebo/physics/physics.hh>
#undef protected
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{

class DeepSimGazeboSystemPluginPrivate;

class DeepSimGazeboSystemPlugin : public SystemPlugin
{
public:
  /// Constructor
  DeepSimGazeboSystemPlugin();

  /// Destructor
  virtual ~DeepSimGazeboSystemPlugin();

  // Documentation inherited
  void Load(int argc, char ** argv) override;

private:
  std::unique_ptr<DeepSimGazeboSystemPluginPrivate> impl_;
};

}  // namespace gazebo_plugins
#endif  // DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_SYSTEM_PLUGIN_HPP_
