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

#include "deepsim_gazebo_plugin/deepsim_gazebo_state_services.hpp"

#include <memory>
#include <vector>
#include <string>

#include "deepsim_gazebo_plugin/deepsim_gazebo_state.hpp"

namespace deepsim_gazebo_plugin
{
class DeepsimGazeboStateServicesPrivate
{
public:
  /// \brief Pointer to the world physicx mutex lock.
  std::mutex & world_physics_lock_;

  /// \brief The call back queue for ros services
  ros::CallbackQueue & ros_callback_queue_;

  /// \brief Helper Gazebo State class for managing gazebo state related operations.
  deepsim_gazebo_plugin::DeepsimGazeboState gazebo_state_;

  /// \brief Contructor that binds mutex lock.
  /// to protect variables accessed on callbacks.
  /// \param[in] _lock std::mutex& pointer to the mutex lock
  explicit DeepsimGazeboStateServicesPrivate(
    std::mutex & _lock,
    ros::CallbackQueue & ros_callback_queue_)
  : world_physics_lock_(_lock),
    ros_callback_queue_(ros_callback_queue_),
    gazebo_state_(_lock) {}

  /// \brief Callback for get model states service.
  /// \param[in] _req Request deepsim_msgs::srv::GetModelStates::Request
  /// \param[out] _res Response deepsim_msgs::srv::GetModelStates::Response
  bool GetModelStates(
    const deepsim_msgs::GetModelStates::Request &_req,
    deepsim_msgs::GetModelStates::Response &_res);

  /// \brief Callback for get link states service.
  /// \param[in] _req Request deepsim_msgs::srv::GetLinkStates::Request::SharedPtr
  /// \param[out] _res Response deepsim_msgs::srv::GetLinkStates::Response::SharedPtr
  bool GetLinkStates(
    const deepsim_msgs::GetLinkStates::Request &_req,
    deepsim_msgs::GetLinkStates::Response &_res);

  /// \brief Callback for get all model states service.
  /// \param[in] _req Request deepsim_msgs::srv::GetAllModelStates::Request
  /// \param[out] _res Response deepsim_msgs::srv::GetAllModelStates::Response
  bool GetAllModelStates(
    const deepsim_msgs::GetAllModelStates::Request &_req,
    deepsim_msgs::GetAllModelStates::Response &_res);

  /// \brief Callback for get all link states service.
  /// \param[in] _req Request deepsim_msgs::srv::GetAllLinkStates::Request::SharedPtr
  /// \param[out] _res Response deepsim_msgs::srv::GetAllLinkStates::Response::SharedPtr
  bool GetAllLinkStates(
    const deepsim_msgs::GetAllLinkStates::Request &_req,
    deepsim_msgs::GetAllLinkStates::Response &_res);

  /// \brief Callback for set model states service.
  /// \param[in] _req Request deepsim_msgs::srv::SetModelStates::Request
  /// \param[out] _res Response deepsim_msgs::srv::SetModelStates::Response
  bool SetModelStates(
    const deepsim_msgs::SetModelStates::Request &_req,
    deepsim_msgs::SetModelStates::Response &_res);

  /// \brief Callback for set link states service.
  /// \param[in] _req Request deepsim_msgs::srv::SetLinkStates::Request
  /// \param[out] _res Response deepsim_msgs::srv::SetLinkStates::Response
  bool SetLinkStates(
    const deepsim_msgs::SetLinkStates::Request &_req,
    deepsim_msgs::SetLinkStates::Response &_res);

  /// \brief ROS service to handle requests for get model states.
  ros::ServiceServer get_model_states_service_;

  /// \brief ROS service to handle requests for get link states.
  ros::ServiceServer get_link_states_service_;

  /// \brief ROS service to handle requests for get all model states.
  ros::ServiceServer get_all_model_states_service_;

  /// \brief ROS service to handle requests for get all link states.
  ros::ServiceServer get_all_link_states_service_;

  /// \brief ROS service to handle requests to set model states.
  ros::ServiceServer set_model_states_service_;

  /// \brief ROS service to handle requests to set link states.
  ros::ServiceServer set_link_states_service_;

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  boost::shared_ptr<ros::NodeHandle> ros_node_;
};

DeepsimGazeboStateServices::DeepsimGazeboStateServices(
  std::mutex & _lock,
  ros::CallbackQueue & ros_callback_queue_)
: impl_(std::make_unique<DeepsimGazeboStateServicesPrivate>(
  _lock, ros_callback_queue_))
{
}

DeepsimGazeboStateServices::~DeepsimGazeboStateServices()
{
}

void DeepsimGazeboStateServices::CreateServices(
  const boost::shared_ptr<ros::NodeHandle> _ros_node,
  const gazebo::physics::WorldPtr _world)
{
  impl_->ros_node_ = _ros_node;
  impl_->world_ = _world;

  ros::AdvertiseServiceOptions get_model_states_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::GetModelStates>(
          "get_model_states",
          boost::bind(
            &DeepsimGazeboStateServicesPrivate::GetModelStates,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->get_model_states_service_ = impl_->ros_node_->advertiseService(get_model_states_aso);

  ros::AdvertiseServiceOptions get_link_states_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::GetLinkStates>(
          "get_link_states",
          boost::bind(
            &DeepsimGazeboStateServicesPrivate::GetLinkStates,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->get_link_states_service_ = impl_->ros_node_->advertiseService(get_link_states_aso);

  ros::AdvertiseServiceOptions get_all_model_states_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::GetAllModelStates>(
          "get_all_model_states",
          boost::bind(
            &DeepsimGazeboStateServicesPrivate::GetAllModelStates,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->get_all_model_states_service_ = impl_->ros_node_->advertiseService(get_all_model_states_aso);

  ros::AdvertiseServiceOptions get_all_link_states_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::GetAllLinkStates>(
          "get_all_link_states",
          boost::bind(
            &DeepsimGazeboStateServicesPrivate::GetAllLinkStates,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->get_all_link_states_service_ = impl_->ros_node_->advertiseService(get_all_link_states_aso);

  ros::AdvertiseServiceOptions set_model_states_aso =
    ros::AdvertiseServiceOptions::create<deepsim_msgs::SetModelStates>(
        "set_model_states",
        boost::bind(
          &DeepsimGazeboStateServicesPrivate::SetModelStates,
          impl_.get(), _1, _2),
        ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->set_model_states_service_ = impl_->ros_node_->advertiseService(set_model_states_aso);

  ros::AdvertiseServiceOptions set_link_states_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::SetLinkStates>(
          "set_link_states",
          boost::bind(
            &DeepsimGazeboStateServicesPrivate::SetLinkStates,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->set_link_states_service_ = impl_->ros_node_->advertiseService(set_link_states_aso);

  ROS_INFO_NAMED(
    "deepsim_gazebo_state_services",
    "[deepsim_gazebo_state_services] State Services Created.");
}

bool DeepsimGazeboStateServicesPrivate::GetModelStates(
  const deepsim_msgs::GetModelStates::Request &_req,
  deepsim_msgs::GetModelStates::Response &_res)
{
  if ( _req.model_names.size() != _req.relative_entity_names.size() )
  {
    _res.status_message = "GetModelStates: model_names, relative_entity_names must be same size!";
    _res.success = false;
    return true;
  }

  std::vector<std::string>::const_iterator model_name_iter = _req.model_names.begin();
  std::vector<std::string>::const_iterator relative_entity_name_iter =
    _req.relative_entity_names.begin();

  for (; model_name_iter != _req.model_names.end();
    ++model_name_iter, ++relative_entity_name_iter)
  {
    // Target entity
    std::string model_name = *model_name_iter;
    std::string relative_entity_name = *relative_entity_name_iter;
    auto model = world_->ModelByName(model_name);
    if (!model)
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_state_services",
        "GetModelStates: one of the entities [%s] does not exist",
        model_name.c_str());
      // An empty placeholder model state to make sure the returned model states list
      // is not empty.
      gazebo_msgs::ModelState empty_model_state;
      empty_model_state.model_name = model_name;
      empty_model_state.reference_frame = relative_entity_name;
      _res.model_states.push_back(empty_model_state);
      _res.status.push_back(false);
      _res.messages.push_back("GetModelStates: model does not exist");
      continue;
    }
    // Frame of reference
    if (relative_entity_name == "" || relative_entity_name == "world"
        || relative_entity_name == "map" || relative_entity_name == "/map")
    {
      ROS_DEBUG_NAMED(
        "deepsim_gazebo_state_services",
        "GetModelStates: reference_frame [%s] is empty/world/map, using inertial frame",
        relative_entity_name.c_str());
    }
    else
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_state_services",
        "GetModelStates: reference_frame [%s] not found, scope the entity name.",
        relative_entity_name.c_str());
      gazebo_msgs::ModelState empty_model_state;
      empty_model_state.model_name = model_name;
      empty_model_state.reference_frame = relative_entity_name;
      _res.model_states.push_back(empty_model_state);
      _res.status.push_back(false);
      _res.messages.push_back("GetModelStates: reference entity not found, scope the entity name.");
      continue;
    }
    auto frame = world_->EntityByName(relative_entity_name);
    auto model_state = gazebo_state_.GetModelState(model, frame);
    _res.model_states.push_back(model_state);
    ROS_DEBUG_NAMED(
      "deepsim_gazebo_state_services",
      "GetModelStates: get model state for [%s]",
      model_state.model_name.c_str());
    _res.status.push_back(true);
    _res.messages.push_back("GetModelState: successful");
  }
  _res.success = true;
  _res.status_message = "GetModelStates: finished";
  return true;
}

bool DeepsimGazeboStateServicesPrivate::GetLinkStates(
  const deepsim_msgs::GetLinkStates::Request &_req,
  deepsim_msgs::GetLinkStates::Response &_res)
{
  if ( _req.link_names.size() != _req.relative_entity_names.size() )
  {
    _res.status_message = "GetLinkStates: link_names, relative_entity_names must be same size!";
    _res.success = false;
    return true;
  }

  std::vector<std::string>::const_iterator link_name_iter = _req.link_names.begin();
  std::vector<std::string>::const_iterator relative_entity_name_iter =
    _req.relative_entity_names.begin();

  for (; link_name_iter != _req.link_names.end();
    ++link_name_iter, ++relative_entity_name_iter)
  {
    // Target entity
    std::string link_name = *link_name_iter;
    std::string relative_entity_name = *relative_entity_name_iter;
    gazebo::physics::LinkPtr link = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(link_name));
    if (!link)
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_state_services",
        "GetLinkStates: one of the entities [%s] does not exist", link_name.c_str());
      // An empty placeholder model state to make sure the returned model states list
      // is not empty.
      gazebo_msgs::LinkState empty_link_state;
      empty_link_state.link_name = link_name;
      empty_link_state.reference_frame = relative_entity_name;
      _res.link_states.push_back(empty_link_state);
      _res.status.push_back(false);
      _res.messages.push_back("GetLinkStates: link does not exist");
      continue;
    }
    // Frame of reference
    if (relative_entity_name == "" || relative_entity_name == "world"
        || relative_entity_name == "map" || relative_entity_name == "/map")
    {
      ROS_DEBUG_NAMED(
        "deepsim_gazebo_state_services",
        "GetLinkStates: reference_frame [%s] is empty/world/map, using inertial frame",
        relative_entity_name.c_str());
    }
    else
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_state_services",
        "GetLinkStates: reference_frame [%s] not found, scope the entity name.",
        relative_entity_name.c_str());
      gazebo_msgs::LinkState empty_link_state;
      empty_link_state.link_name = link_name;
      empty_link_state.reference_frame = relative_entity_name;
      _res.link_states.push_back(empty_link_state);
      _res.status.push_back(false);
      _res.messages.push_back("GetLinkStates: reference entity not found, scope the entity name.");
      continue;
    }
    auto frame = world_->EntityByName(relative_entity_name);
    auto link_state = gazebo_state_.GetLinkState(link, frame);
    _res.link_states.push_back(link_state);
    ROS_DEBUG_NAMED(
      "deepsim_gazebo_state_services",
      "GetLinkStates: get link state for [%s]",
      link_state.link_name.c_str());
    _res.status.push_back(true);
    _res.messages.push_back("GetLinkState: successful");
  }
  _res.success = true;
  _res.status_message = "GetLinkStates: Finished";
  return true;
}

bool DeepsimGazeboStateServicesPrivate::GetAllModelStates(
  const deepsim_msgs::GetAllModelStates::Request &_req,
  deepsim_msgs::GetAllModelStates::Response &_res)
{
  // fill model_states
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->ModelByIndex(i);
    ignition::math::Pose3d  model_pose = model->WorldPose(); // - myBody->GetCoMPose();
    ignition::math::Vector3d linear_vel  = model->WorldLinearVel();
    ignition::math::Vector3d angular_vel = model->WorldAngularVel();
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->GetModel(i);
    ignition::math::Pose3d  model_pose = model->GetWorldPose().Ign(); // - myBody->GetCoMPose();
    ignition::math::Vector3d linear_vel  = model->GetWorldLinearVel().Ign();
    ignition::math::Vector3d angular_vel = model->GetWorldAngularVel().Ign();
#endif
    ignition::math::Vector3d pos = model_pose.Pos();
    ignition::math::Quaterniond rot = model_pose.Rot();
    geometry_msgs::Pose pose;
    pose.position.x = pos.X();
    pose.position.y = pos.Y();
    pose.position.z = pos.Z();
    pose.orientation.w = rot.W();
    pose.orientation.x = rot.X();
    pose.orientation.y = rot.Y();
    pose.orientation.z = rot.Z();

    geometry_msgs::Twist twist;
    twist.linear.x = linear_vel.X();
    twist.linear.y = linear_vel.Y();
    twist.linear.z = linear_vel.Z();
    twist.angular.x = angular_vel.X();
    twist.angular.y = angular_vel.Y();
    twist.angular.z = angular_vel.Z();

    gazebo_msgs::ModelState model_state;
    model_state.model_name = model->GetName();
    model_state.pose = pose;
    model_state.twist = twist;
    _res.model_states.push_back(model_state);
  }
  _res.success = true;
  _res.status_message = "GetAllModelStates: finished";
  return true;
}

bool DeepsimGazeboStateServicesPrivate::GetAllLinkStates(
  const deepsim_msgs::GetAllLinkStates::Request &_req,
  deepsim_msgs::GetAllLinkStates::Response &_res)
{
  // fill link_states
#if GAZEBO_MAJOR_VERSION >= 8
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->ModelByIndex(i);
#else
  for (unsigned int i = 0; i < world_->GetModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->GetModel(i);
#endif

    for (unsigned int j = 0 ; j < model->GetChildCount(); j ++)
    {
      gazebo::physics::LinkPtr body = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));

      if (body)
      {

        geometry_msgs::Pose pose;
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d  body_pose = body->WorldPose(); // - myBody->GetCoMPose();
        ignition::math::Vector3d linear_vel  = body->WorldLinearVel();
        ignition::math::Vector3d angular_vel = body->WorldAngularVel();
#else
        ignition::math::Pose3d  body_pose = body->GetWorldPose().Ign(); // - myBody->GetCoMPose();
        ignition::math::Vector3d linear_vel  = body->GetWorldLinearVel().Ign();
        ignition::math::Vector3d angular_vel = body->GetWorldAngularVel().Ign();
#endif
        ignition::math::Vector3d pos = body_pose.Pos();
        ignition::math::Quaterniond rot = body_pose.Rot();
        pose.position.x = pos.X();
        pose.position.y = pos.Y();
        pose.position.z = pos.Z();
        pose.orientation.w = rot.W();
        pose.orientation.x = rot.X();
        pose.orientation.y = rot.Y();
        pose.orientation.z = rot.Z();

        geometry_msgs::Twist twist;
        twist.linear.x = linear_vel.X();
        twist.linear.y = linear_vel.Y();
        twist.linear.z = linear_vel.Z();
        twist.angular.x = angular_vel.X();
        twist.angular.y = angular_vel.Y();
        twist.angular.z = angular_vel.Z();

        gazebo_msgs::LinkState link_state;
        link_state.link_name = body->GetScopedName();
        link_state.pose = pose;
        link_state.twist = twist;
        _res.link_states.push_back(link_state);
      }
    }
  }
  _res.success = true;
  _res.status_message = "GetAllLinkStates: Finished";
  return true;
}

bool DeepsimGazeboStateServicesPrivate::SetModelStates(
  const deepsim_msgs::SetModelStates::Request &_req,
  deepsim_msgs::SetModelStates::Response &_res)
{
  std::vector<gazebo_msgs::ModelState>::const_iterator model_state_iter = _req.model_states.begin();

  std::vector<deepsim_gazebo_plugin::EntityStateInfo> entity_state_info_list;
  std::vector<gazebo::physics::ModelPtr> target_models;

  for (; model_state_iter != _req.model_states.end(); ++model_state_iter)
  {
    gazebo_msgs::ModelState model_state = *model_state_iter;
    // Target entity
    gazebo::physics::ModelPtr target_model = world_->ModelByName(model_state.model_name);
    if (!target_model)
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_state_services",
        "SetModelStates: model [%s] does not exist", model_state.model_name.c_str());
      _res.status.push_back(false);
      _res.messages.push_back("SetModelStates: model does not exist");
      continue;
    }

    // Frame of reference
    auto frame = world_->EntityByName(model_state.reference_frame);
    if (model_state.reference_frame == "" || model_state.reference_frame == "world"
      || model_state.reference_frame == "map" || model_state.reference_frame == "/map")
    {
      ROS_DEBUG_NAMED(
        "deepsim_gazebo_state_services",
        "SetModelStates: reference_frame [%s] is empty/world/map, using inertial frame",
        model_state.reference_frame.c_str());
    }
    else
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_state_services",
        "SetModelStates: reference entity [%s] not found, scope the entity name.",
        model_state.reference_frame.c_str());
      _res.status.push_back(false);
      _res.messages.push_back("SetModelStates: reference entity not found, scope the entity name.");
      continue;
    }
    // Get Entity State Info
    auto entity_state_info = gazebo_state_.GetInputEntityStateInfo(model_state, frame);
    bool set_vel_status = gazebo_state_.SetVelocity(entity_state_info, target_model);
    entity_state_info_list.push_back(entity_state_info);
    target_models.push_back(target_model);
    _res.status.push_back(set_vel_status);
    _res.messages.push_back("Finished setting velocity.");
  }

  // This part contains a critical section that pauses and unpauses the world.
  bool set_pose_status = gazebo_state_.SetPoses(entity_state_info_list, target_models, world_);

  ROS_DEBUG_NAMED(
    "deepsim_gazebo_state_services",
    "SetModelStates: Finished.");
  _res.success = set_pose_status;
  _res.status_message = set_pose_status ? "SetLinkStates: Finished" : "SetLinkStates: Failed";
  return true;
}

bool DeepsimGazeboStateServicesPrivate::SetLinkStates(
  const deepsim_msgs::SetLinkStates::Request &_req,
  deepsim_msgs::SetLinkStates::Response &_res)
{
  std::vector<gazebo_msgs::LinkState>::const_iterator link_state_iter = _req.link_states.begin();

  std::vector<deepsim_gazebo_plugin::EntityStateInfo> entity_state_info_list;
  std::vector<gazebo::physics::LinkPtr> target_links;

  for (; link_state_iter != _req.link_states.end(); ++link_state_iter)
  {
    gazebo_msgs::LinkState link_state = *link_state_iter;
    // Target entity
    gazebo::physics::LinkPtr target_link =
      boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(link_state.link_name));
    if (!target_link)
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_state_services",
        "SetLinkStates: link [%s] does not exist", link_state.link_name.c_str());
      _res.status.push_back(false);
      _res.messages.push_back("SetLinkStates: link does not exist");
      continue;
    }

    // Frame of reference
    auto frame = world_->EntityByName(link_state.reference_frame);
    if (link_state.reference_frame == "" || link_state.reference_frame == "world"
      || link_state.reference_frame == "map" || link_state.reference_frame == "/map")
    {
      ROS_DEBUG_NAMED(
        "deepsim_gazebo_state_services",
        "SetLinkStates: reference_frame [%s] is empty/world/map, using inertial frame",
        link_state.reference_frame.c_str());
    }
    else
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_state_services",
        "SetLinkStates: reference entity [%s] not found, scope the entity name.",
        link_state.reference_frame.c_str());
      _res.status.push_back(false);
      _res.messages.push_back("SetLinkStates: reference entity not found, scope the entity name.");
      continue;
    }
    // Get Entity State Info
    auto entity_state_info = gazebo_state_.GetInputEntityStateInfo(link_state, frame);
    bool set_vel_status = gazebo_state_.SetVelocity(entity_state_info, target_link);
    entity_state_info_list.push_back(entity_state_info);
    target_links.push_back(target_link);
    _res.status.push_back(set_vel_status);
    _res.messages.push_back("Finished setting velocity.");
  }

  // This part contains a critical section that pauses and unpauses the world.
  bool set_pose_status = gazebo_state_.SetPoses(entity_state_info_list, target_links, world_);

  ROS_DEBUG_NAMED(
    "deepsim_gazebo_state_services",
    "SetLinkStates: Finished.");

  _res.success = set_pose_status;
  _res.status_message = set_pose_status ? "SetLinkStates: Finished" : "SetLinkStates: Failed";
  return true;
}
}  // namespace deepsim_gazebo_plugin
