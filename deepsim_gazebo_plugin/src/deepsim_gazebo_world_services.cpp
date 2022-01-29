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

#include <deepsim_gazebo_plugin/deepsim_gazebo_world_services.hpp>

#include <memory>
#include <string>
#include <vector>
namespace deepsim_gazebo_plugin
{
class DeepsimGazeboWorldServicesPrivate
{
public:
  /// \brief Pointer to the world physicx mutex lock.
  std::mutex & world_physics_lock_;

  /// \brief The call back queue for ros services
  ros::CallbackQueue & ros_callback_queue_;

  /// \brief Contructor that binds mutex lock.
  /// to protect variables accessed on callbacks.
  /// \param[in] _lock std::mutex& pointer to the mutex lock
  explicit DeepsimGazeboWorldServicesPrivate(
    std::mutex & _lock,
    ros::CallbackQueue & ros_callback_queue_)
  : world_physics_lock_(_lock),
    ros_callback_queue_(ros_callback_queue_) {}

  /// Callback from ROS service to pause physics.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  bool OnPause(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res);

  /// Callback from ROS service to unpause (play) physics.
  /// \param[in] req Empty request
  /// \param[out] res Empty response
  bool OnUnpause(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res);

  /// Callback from ROS service to Get all the light names.
  /// \param[in] req deepsim_msgs::GetLightNames request
  /// \param[out] res deepsim_msgs::GetLightNames response
  bool GetLightNames(
    deepsim_msgs::GetLightNames::Request &req,
    deepsim_msgs::GetLightNames::Response &_res);

  /// \brief ROS service to handle requests/responses to pause world physics.
  ros::ServiceServer pause_world_physics_service_;

  /// \brief ROS service to handle requests/responses to unpause world physics.
  ros::ServiceServer unpause_world_physics_service_;

  /// \brief ROS service to handle requests/responses to get light names.
  ros::ServiceServer get_light_names_service_;

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// \brief ROS node for communication, managed by gazebo_ros.
  boost::shared_ptr<ros::NodeHandle> ros_node_;
};

DeepsimGazeboWorldServices::DeepsimGazeboWorldServices(
  std::mutex & _lock,
  ros::CallbackQueue & ros_callback_queue_)
: impl_(std::make_unique<DeepsimGazeboWorldServicesPrivate>(
  _lock, ros_callback_queue_))
{
}

DeepsimGazeboWorldServices::~DeepsimGazeboWorldServices()
{
}

void DeepsimGazeboWorldServices::CreateServices(
  const boost::shared_ptr<ros::NodeHandle> _ros_node,
  const gazebo::physics::WorldPtr _world)
{
  impl_->ros_node_ = _ros_node;
  impl_->world_ = _world;
  // ros services
  ros::AdvertiseServiceOptions pause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      "pause_world_physics",
      boost::bind(
        &DeepsimGazeboWorldServicesPrivate::OnPause, impl_.get(),
        _1, _2),
      ros::VoidPtr(),
      &impl_->ros_callback_queue_);
  impl_->pause_world_physics_service_ = impl_->ros_node_->advertiseService(pause_physics_aso);

  ros::AdvertiseServiceOptions unpause_physics_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      "unpause_world_physics",
      boost::bind(
        &DeepsimGazeboWorldServicesPrivate::OnUnpause, impl_.get(),
        _1, _2),
      ros::VoidPtr(),
      &impl_->ros_callback_queue_);
  impl_->unpause_world_physics_service_ = impl_->ros_node_->advertiseService(unpause_physics_aso);

  ros::AdvertiseServiceOptions get_light_names_aso =
    ros::AdvertiseServiceOptions::create<deepsim_msgs::GetLightNames>(
      "get_light_names",
      boost::bind(
        &DeepsimGazeboWorldServicesPrivate::GetLightNames, impl_.get(),
        _1, _2),
      ros::VoidPtr(),
      &impl_->ros_callback_queue_);
  impl_->get_light_names_service_ = impl_->ros_node_->advertiseService(get_light_names_aso);

  ROS_INFO_NAMED(
    "deepsim_gazebo_world_services",
    "[deepsim_gazebo_world_services] World Services Created.");
}

bool DeepsimGazeboWorldServicesPrivate::OnPause(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  std::lock_guard<std::mutex> lock(world_physics_lock_);
  world_->SetPaused(true);
  ROS_DEBUG_NAMED(
    "deepsim_gazebo_world_services",
    "[deepsim_gazebo_world_services] Gazebo World Paused.");
  return true;
}

bool DeepsimGazeboWorldServicesPrivate::OnUnpause(
  std_srvs::Empty::Request &req,
  std_srvs::Empty::Response &res)
{
  std::lock_guard<std::mutex> lock(world_physics_lock_);
  world_->SetPaused(false);
  ROS_DEBUG_NAMED(
    "deepsim_gazebo_world_services",
    "[deepsim_gazebo_world_services] Gazebo World Unpaused.");
  return true;
}

bool DeepsimGazeboWorldServicesPrivate::GetLightNames(
  deepsim_msgs::GetLightNames::Request &req,
  deepsim_msgs::GetLightNames::Response &_res)
{
  std::vector<std::string> lightNames;

  gazebo::physics::Light_V lights = world_->Lights();
  for (gazebo::physics::Light_V::const_iterator lightIter = lights.begin();
      lightIter != lights.end();
      ++lightIter)
  {
    lightNames.push_back(boost::dynamic_pointer_cast<gazebo::physics::Base>(*lightIter)->GetName());
  }
  _res.light_names = lightNames;
  _res.success = true;
  ROS_INFO_NAMED(
    "deepsim_gazebo_world_services",
    "[deepsim_gazebo_world_services] GetLightNames: Finished.");
  return true;
}
}  // namespace deepsim_gazebo_plugin
