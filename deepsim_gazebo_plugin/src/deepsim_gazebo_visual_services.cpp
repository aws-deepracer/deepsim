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

#include "deepsim_gazebo_plugin/deepsim_gazebo_visual_services.hpp"

#include <memory>
#include <vector>
#include <string>

namespace deepsim_gazebo_plugin
{
class DeepsimGazeboVisualServicesPrivate
{
public:
  /// \brief Pointer to the world physicx mutex lock.
  std::mutex & visuals_lock;

  /// \brief The call back queue for ros services
  ros::CallbackQueue & ros_callback_queue_;

  /// \brief
  deepsim_gazebo_plugin::DeepsimGazeboVisualPublisher visual_publisher_;

  /// \brief Contructor that binds mutex lock.
  /// to protect variables accessed on callbacks.
  /// \param[in] _lock std::mutex& pointer to the mutex lock
  /// \param[in] _ros_callback_queue ros::CallbackQueue call back queue for gz services
  explicit DeepsimGazeboVisualServicesPrivate(
    std::mutex & _lock,
    ros::CallbackQueue & _ros_callback_queue)
  : visuals_lock(_lock),
    ros_callback_queue_(_ros_callback_queue),
    visual_publisher_(ros_callback_queue_) {}

  /// \brief Callback for get visual names service.
  /// \param[in] _req Request deepsim_msgs::srv::GetVisualNames::Request
  /// \param[out] _res Response deepsim_msgs::srv::GetVisualNames::Response
  bool GetVisualNames(
    const deepsim_msgs::GetVisualNames::Request &_req,
    deepsim_msgs::GetVisualNames::Response &_res);

  /// \brief Callback for get visual service.
  /// \param[in] _req Request deepsim_msgs::srv::GetVisual::Request
  /// \param[out] _res Response deepsim_msgs::srv::GetVisual::Response
  bool GetVisual(
    const deepsim_msgs::GetVisual::Request &_req,
    deepsim_msgs::GetVisual::Response &_res);

  /// \brief Callback for get visuals service.
  /// \param[in] _req Request deepsim_msgs::srv::GetVisuals::Request
  /// \param[out] _res Response deepsim_msgs::srv::GetVisuals::Response
  bool GetVisuals(
    const deepsim_msgs::GetVisuals::Request &_req,
    deepsim_msgs::GetVisuals::Response &_res);

  /// \brief Callback for get all visuals service.
  /// \param[in] _req Request deepsim_msgs::srv::GetAllVisuals::Request
  /// \param[out] _res Response deepsim_msgs::srv::GetAllVisuals::Response
  bool GetAllVisuals(
    const deepsim_msgs::GetAllVisuals::Request &_req,
    deepsim_msgs::GetAllVisuals::Response &_res);

  /// \brief Callback for set visual material service.
  /// \param[in] _req Request deepsim_msgs::srv::SetVisualMaterial::Request::SharedPtr
  /// \param[out] _res Response deepsim_msgs::srv::SetVisualMaterial::Response::SharedPtr
  bool SetVisualMaterial(
    const deepsim_msgs::SetVisualMaterial::Request &_req,
    deepsim_msgs::SetVisualMaterial::Response &_res);

  /// \brief Callback for set visual materials service.
  /// \param[in] _req Request deepsim_msgs::srv::SetVisualMaterials::Request::SharedPtr
  /// \param[out] _res Response deepsim_msgs::srv::SetVisualMaterials::Response::SharedPtr
  bool SetVisualMaterials(
    const deepsim_msgs::SetVisualMaterials::Request &_req,
    deepsim_msgs::SetVisualMaterials::Response &_res);

  /// \brief Callback for set visual transparency service.
  /// \param[in] _req Request deepsim_msgs::srv::SetVisualTransparency::Request
  /// \param[out] _res Response deepsim_msgs::srv::SetVisualTransparency::Response
  bool SetVisualTransparency(
    const deepsim_msgs::SetVisualTransparency::Request &_req,
    deepsim_msgs::SetVisualTransparency::Response &_res);

  /// \brief Callback for set visual transparencies service.
  /// \param[in] _req Request deepsim_msgs::srv::SetVisualTransparencies::Request
  /// \param[out] _res Response deepsim_msgs::srv::SetVisualTransparencies::Response
  bool SetVisualTransparencies(
    const deepsim_msgs::SetVisualTransparencies::Request &_req,
    deepsim_msgs::SetVisualTransparencies::Response &_res);

  /// \brief Callback for set visual visible service.
  /// \param[in] _req Request deepsim_msgs::srv::SetVisualVisible::Request
  /// \param[out] _res Response deepsim_msgs::srv::SetVisualVisible::Response
  bool SetVisualVisible(
    const deepsim_msgs::SetVisualVisible::Request &_req,
    deepsim_msgs::SetVisualVisible::Response &_res);

  /// \brief Callback for set visual visibles service.
  /// \param[in] _req Request deepsim_msgs::srv::SetVisualVisibles::Request
  /// \param[out] _res Response deepsim_msgs::srv::SetVisualVisibles::Response
  bool SetVisualVisibles(
    const deepsim_msgs::SetVisualVisibles::Request &_req,
    deepsim_msgs::SetVisualVisibles::Response &_res);

  /// \brief ROS service to handle requests for getting visual names.
  ros::ServiceServer get_visual_names_service_;

  /// \brief ROS service to handle requests for getting visual.
  ros::ServiceServer get_visual_service_;

  /// \brief ROS service to handle requests for getting visuals.
  ros::ServiceServer get_visuals_service_;

  /// \brief ROS service to handle requests for getting all visuals.
  ros::ServiceServer get_all_visuals_service_;

  /// \brief ROS service to handle requests to set visual material.
  ros::ServiceServer set_visual_material_service_;

  /// \brief ROS service to handle requests to set visual materials.
  ros::ServiceServer set_visual_materials_service_;

  /// \brief ROS service to handle requests to set visual transparenciy.
  ros::ServiceServer set_visual_transparency_service_;

  /// \brief ROS service to handle requests to set visual transparencies.
  ros::ServiceServer set_visual_transparencies_service_;

  /// \brief ROS service to handle requests to set visual visible.
  ros::ServiceServer set_visual_visible_service_;

  /// \brief ROS service to handle requests to set visual visibles.
  ros::ServiceServer set_visual_visibles_service_;

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  boost::shared_ptr<ros::NodeHandle> ros_node_;
};

DeepsimGazeboVisualServices::DeepsimGazeboVisualServices(
  std::mutex & _lock,
  ros::CallbackQueue & _ros_callback_queue)
: impl_(std::make_unique<DeepsimGazeboVisualServicesPrivate>(
  _lock, _ros_callback_queue))
{
}

DeepsimGazeboVisualServices::~DeepsimGazeboVisualServices()
{
}

void DeepsimGazeboVisualServices::CreateServices(
  const gazebo::transport::NodePtr _gz_node,
  const boost::shared_ptr<ros::NodeHandle> _ros_node,
  const gazebo::physics::WorldPtr _world)
{
  impl_->ros_node_ = _ros_node;
  impl_->world_ = _world;

  // Create Publisher
  impl_->visual_publisher_.CreatePublisher(_gz_node, _ros_node, _world);

  ros::AdvertiseServiceOptions get_visual_names_aso =
    ros::AdvertiseServiceOptions::create<deepsim_msgs::GetVisualNames>(
          "get_visual_names",
          boost::bind(
            &DeepsimGazeboVisualServicesPrivate::GetVisualNames,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->get_visual_names_service_ = impl_->ros_node_->advertiseService(get_visual_names_aso);

  ros::AdvertiseServiceOptions get_visual_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::GetVisual>(
          "get_visual",
          boost::bind(
            &DeepsimGazeboVisualServicesPrivate::GetVisual,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->get_visual_service_ = impl_->ros_node_->advertiseService(get_visual_aso);

  ros::AdvertiseServiceOptions get_visuals_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::GetVisuals>(
          "get_visuals",
          boost::bind(
            &DeepsimGazeboVisualServicesPrivate::GetVisuals,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->get_visuals_service_ = impl_->ros_node_->advertiseService(get_visuals_aso);

ros::AdvertiseServiceOptions get_all_visuals_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::GetAllVisuals>(
          "get_all_visuals",
          boost::bind(
            &DeepsimGazeboVisualServicesPrivate::GetAllVisuals,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->get_all_visuals_service_ = impl_->ros_node_->advertiseService(get_all_visuals_aso);

  ros::AdvertiseServiceOptions set_visual_material_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::SetVisualMaterial>(
          "set_visual_material",
          boost::bind(
            &DeepsimGazeboVisualServicesPrivate::SetVisualMaterial,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->set_visual_material_service_ = impl_->ros_node_->advertiseService(set_visual_material_aso);

    ros::AdvertiseServiceOptions set_visual_materials_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::SetVisualMaterials>(
          "set_visual_materials",
          boost::bind(
            &DeepsimGazeboVisualServicesPrivate::SetVisualMaterials,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->set_visual_materials_service_ = impl_->ros_node_->advertiseService(set_visual_materials_aso);

  ros::AdvertiseServiceOptions set_visual_transparency_aso =
    ros::AdvertiseServiceOptions::create<deepsim_msgs::SetVisualTransparency>(
        "set_visual_transparency",
        boost::bind(
          &DeepsimGazeboVisualServicesPrivate::SetVisualTransparency,
          impl_.get(), _1, _2),
        ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->set_visual_transparency_service_ = impl_->ros_node_->advertiseService(set_visual_transparency_aso);

  ros::AdvertiseServiceOptions set_visual_transparencies_aso =
    ros::AdvertiseServiceOptions::create<deepsim_msgs::SetVisualTransparencies>(
        "set_visual_transparencies",
        boost::bind(
          &DeepsimGazeboVisualServicesPrivate::SetVisualTransparencies,
          impl_.get(), _1, _2),
        ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->set_visual_transparencies_service_ = impl_->ros_node_->advertiseService(set_visual_transparencies_aso);

  ros::AdvertiseServiceOptions set_visual_visible_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::SetVisualVisible>(
          "set_visual_visible",
          boost::bind(
            &DeepsimGazeboVisualServicesPrivate::SetVisualVisible,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->set_visual_visible_service_ = impl_->ros_node_->advertiseService(set_visual_visible_aso);

  ros::AdvertiseServiceOptions set_visual_visibles_aso =
      ros::AdvertiseServiceOptions::create<deepsim_msgs::SetVisualVisibles>(
          "set_visual_visibles",
          boost::bind(
            &DeepsimGazeboVisualServicesPrivate::SetVisualVisibles,
            impl_.get(), _1, _2),
          ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->set_visual_visibles_service_ = impl_->ros_node_->advertiseService(set_visual_visibles_aso);

  ROS_INFO_NAMED(
    "deepsim_gazebo_visual_services",
    "[deepsim_gazebo_visual_services] Visual Services Created.");
}

bool DeepsimGazeboVisualServicesPrivate::GetVisualNames(
  const deepsim_msgs::GetVisualNames::Request &_req,
  deepsim_msgs::GetVisualNames::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);

  for (std::vector<std::string>::const_iterator link_name_iter = _req.link_names.begin();
      link_name_iter != _req.link_names.end();
      ++link_name_iter)
  {
    std::string link_name = *link_name_iter;
    gazebo::physics::LinkPtr link =
      boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(link_name));
    if (!link)
    {
        ROS_ERROR_NAMED(
          "deepsim_gazebo_visual_services",
          "GetVisualNames: link [%s] does not exist",
          link_name.c_str());
        _res.status_message = "GetVisualNames: link does not exist";
        _res.success = false;
        return true;
    }
    for (gazebo::physics::Link::Visuals_M::const_iterator visual_iter = link->visuals.begin();
          visual_iter != link->visuals.end();
          ++visual_iter)
    {
        _res.visual_names.push_back(visual_iter->second.name());
        _res.link_names.push_back(link_name);
    }
  }
  _res.success = true;
  _res.status_message = "GetVisualNames: Finished.";
  return true;
}

bool DeepsimGazeboVisualServicesPrivate::GetVisual(
  const deepsim_msgs::GetVisual::Request &_req,
  deepsim_msgs::GetVisual::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);

  gazebo::physics::LinkPtr link =
    boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(_req.link_name));
  if (!link)
  {
    ROS_ERROR_NAMED(
      "deepsim_gazebo_visual_services",
      "GetVisual: the link [%s] does not exist",
      _req.link_name.c_str());
    _res.success = false;
    _res.status_message = "GetVisual: link does not exist.";
    return true;
  }

  // visual message
  // TODO(saraxu): need to handle visual_name does not exist situation better.
  // Right now it return a bunch of default values.
  deepsim_msgs::Visual visual = deepsim_gazebo_plugin::VisualConverter::Convert2DeepsimVisual(
    _req.link_name,
    _req.visual_name,
    link->GetVisualMessage(_req.visual_name));
  _res.visual = visual;
  _res.success = true;
  _res.status_message = "GetVisual: Finished.";
  return true;
}

bool DeepsimGazeboVisualServicesPrivate::GetVisuals(
  const deepsim_msgs::GetVisuals::Request &_req,
  deepsim_msgs::GetVisuals::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);

  if ( _req.link_names.size() != _req.visual_names.size() )
  {
    _res.status_message = "GetVisuals: link_names, visual_names must be same size!";
    _res.success = false;
    return true;
  }

  std::vector<std::string>::const_iterator link_name_iter = _req.link_names.begin();
  std::vector<std::string>::const_iterator visual_name_iter = _req.visual_names.begin();

  for (; link_name_iter != _req.link_names.end();
    ++link_name_iter, ++visual_name_iter)
  {
    std::string link_name = *link_name_iter;
    std::string visual_name = *visual_name_iter;

    gazebo::physics::LinkPtr link =
      boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(link_name));
    if (!link)
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_visual_services",
        "GetVisuals: one of the link [%s] does not exist",
        link_name.c_str());
      _res.status.push_back(false);
      _res.messages.push_back("GetVisuals: link does not exist.");
      continue;
    }

    // visual message
    // TODO(saraxu): need to handle visual_name does not exist situation better.
    // Right now it return a bunch of default values.
    deepsim_msgs::Visual visual = deepsim_gazebo_plugin::VisualConverter::Convert2DeepsimVisual(
      link_name,
      visual_name,
      link->GetVisualMessage(visual_name));
    _res.visuals.push_back(visual);
    _res.status.push_back(true);
    _res.messages.push_back("GetVisuals: Success for visual.");
  }
  _res.success = true;
  _res.status_message = "GetVisuals: Finished.";
  return true;
}

bool DeepsimGazeboVisualServicesPrivate::GetAllVisuals(
  const deepsim_msgs::GetAllVisuals::Request &_req,
  deepsim_msgs::GetAllVisuals::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);
  // fill visuals
  for (unsigned int i = 0; i < world_->ModelCount(); i ++)
  {
    gazebo::physics::ModelPtr model = world_->ModelByIndex(i);

    for (unsigned int j = 0 ; j < model->GetChildCount(); j ++)
    {
      gazebo::physics::LinkPtr link = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));

      if (link)
      {
        // fill visuals
        for (gazebo::physics::Link::Visuals_M::const_iterator visual_iter = link->visuals.begin();
          visual_iter != link->visuals.end();
          ++visual_iter)
        {
            deepsim_msgs::Visual visual = deepsim_gazebo_plugin::VisualConverter::Convert2DeepsimVisual(
              link->GetScopedName(),
              visual_iter->second.name(),
              link->GetVisualMessage(visual_iter->second.name()));
            _res.visuals.push_back(visual);
        }
      }
    }
  }
  _res.success = true;
  _res.status_message = "GetAllVisuals: Finished.";
  return true;
}

bool DeepsimGazeboVisualServicesPrivate::SetVisualMaterial(
    const deepsim_msgs::SetVisualMaterial::Request &_req,
    deepsim_msgs::SetVisualMaterial::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);

  gazebo::physics::LinkPtr link =
    boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(_req.link_name));
  if (!link)
  {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_visual_services",
        "SetVisualMaterial: link [%s] does not exist", _req.link_name.c_str());
      _res.success = false;
      _res.status_message = "SetVisualMaterial: link does not exist.";
  }

  gazebo::msgs::Visual visual_msg = link->GetVisualMessage(_req.visual_name);
  // Set colors to material object
  gazebo::msgs::Material *material_msg = visual_msg.mutable_material();
  deepsim_gazebo_plugin::VisualConverter::SetGazeboColor(
    material_msg->mutable_ambient(),
    _req.ambient);
  deepsim_gazebo_plugin::VisualConverter::SetGazeboColor(
    material_msg->mutable_diffuse(),
    _req.diffuse);
  deepsim_gazebo_plugin::VisualConverter::SetGazeboColor(
    material_msg->mutable_specular(),
    _req.specular);
  deepsim_gazebo_plugin::VisualConverter::SetGazeboColor(
    material_msg->mutable_emissive(),
    _req.emissive);
  // Publish visual message
  visual_publisher_.PublishVisualMsg(link, visual_msg, _req.block);
  _res.status_message = "SetVisualMaterial: Success.";
  _res.success = true;
  return true;
}

bool DeepsimGazeboVisualServicesPrivate::SetVisualMaterials(
    const deepsim_msgs::SetVisualMaterials::Request &_req,
    deepsim_msgs::SetVisualMaterials::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);

  if ( _req.link_names.size() != _req.visual_names.size() ||
      _req.link_names.size() != _req.ambients.size() ||
      _req.link_names.size() != _req.diffuses.size() ||
      _req.link_names.size() != _req.speculars.size() ||
      _req.link_names.size() != _req.emissives.size())
  {
      _res.status_message =
        "SetVisualMaterials: link_names, visual_names, ambients, diffuses, speculars, emissives must be same size!";
      _res.success = false;
      return true;
  }
    std::vector<std::string>::const_iterator link_name_iter = _req.link_names.begin();
    std::vector<std::string>::const_iterator visual_name_iter = _req.visual_names.begin();
    std::vector<std_msgs::ColorRGBA>::const_iterator ambient_iter = _req.ambients.begin();
    std::vector<std_msgs::ColorRGBA>::const_iterator diffuse_iter = _req.diffuses.begin();
    std::vector<std_msgs::ColorRGBA>::const_iterator specular_iter = _req.speculars.begin();
    std::vector<std_msgs::ColorRGBA>::const_iterator emissive_iter = _req.emissives.begin();

    for (; link_name_iter != _req.link_names.end();
          ++link_name_iter, ++visual_name_iter, ++ambient_iter, ++diffuse_iter, ++specular_iter, ++emissive_iter)
    {
      std::string link_name = *link_name_iter;
      std::string visual_name = *visual_name_iter;
      gazebo::physics::LinkPtr link =
        boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(link_name));
      if (!link)
      {
          ROS_ERROR_NAMED(
            "deepsim_gazebo_visual_services",
            "SetVisualMaterials: link [%s] does not exist", link_name.c_str());
          _res.status.push_back(false);
          _res.messages.push_back("SetVisualMaterials: link does not exist.");
          continue;
      }
      gazebo::msgs::Visual visual_msg = link->GetVisualMessage(visual_name);
      // Set colors to material object
      gazebo::msgs::Material *material_msg = visual_msg.mutable_material();
      deepsim_gazebo_plugin::VisualConverter::SetGazeboColor(
        material_msg->mutable_ambient(),
        *ambient_iter);
      deepsim_gazebo_plugin::VisualConverter::SetGazeboColor(
        material_msg->mutable_diffuse(),
        *diffuse_iter);
      deepsim_gazebo_plugin::VisualConverter::SetGazeboColor(
        material_msg->mutable_specular(),
        *specular_iter);
      deepsim_gazebo_plugin::VisualConverter::SetGazeboColor(
        material_msg->mutable_emissive(),
        *emissive_iter);
      // Publish visual message
      visual_publisher_.PublishVisualMsg(link, visual_msg, _req.block);
      _res.status.push_back(true);
      _res.messages.push_back("SetVisualMaterials: Success for visual.");
  }
  _res.success = true;
  _res.status_message = "SetVisualMaterial: Finished.";
  return true;
}

bool DeepsimGazeboVisualServicesPrivate::SetVisualTransparency(
    const deepsim_msgs::SetVisualTransparency::Request &_req,
    deepsim_msgs::SetVisualTransparency::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);

  gazebo::physics::LinkPtr link =
    boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(_req.link_name));
  if (!link)
  {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_visual_services",
        "SetVisualTransparency: link [%s] does not exist", _req.link_name.c_str());
      _res.success = false;
      _res.status_message = "SetVisualTransparency: link does not exist.";
  }
  gazebo::msgs::Visual visual_msg = link->GetVisualMessage(_req.visual_name);
  visual_msg.set_transparency(_req.transparency);
  // Publish visual message
  visual_publisher_.PublishVisualMsg(link, visual_msg, _req.block);
  _res.success = true;
  _res.status_message = "SetVisualTransparency: Succcess.";
  return true;
}

bool DeepsimGazeboVisualServicesPrivate::SetVisualTransparencies(
    const deepsim_msgs::SetVisualTransparencies::Request &_req,
    deepsim_msgs::SetVisualTransparencies::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);
  if ( _req.link_names.size() != _req.visual_names.size() ||
    _req.link_names.size() != _req.transparencies.size())
  {
    _res.status_message = "SetVisualTransparencies: link_names, visual_names, transparencies must be same size!";
    _res.success = false;
    return true;
  }
  std::vector<std::string>::const_iterator link_name_iter = _req.link_names.begin();
  std::vector<std::string>::const_iterator visual_name_iter = _req.visual_names.begin();
  std::vector<double>::const_iterator transparency_iter = _req.transparencies.begin();

  for (; link_name_iter != _req.link_names.end();
      ++link_name_iter, ++visual_name_iter, ++transparency_iter)
  {
    std::string link_name = *link_name_iter;
    std::string visual_name = *visual_name_iter;
    gazebo::physics::LinkPtr link = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(link_name));
    if (!link)
    {
        ROS_ERROR_NAMED(
          "deepsim_gazebo_visual_services",
          "SetVisualTransparencies: link [%s] does not exist", link_name.c_str());
        _res.status.push_back(false);
        _res.messages.push_back("SetVisualTransparencies: link does not exist");
        continue;
    }
    gazebo::msgs::Visual visual_msg = link->GetVisualMessage(*visual_name_iter);
    visual_msg.set_transparency(*transparency_iter);
    // Publish visual message
    visual_publisher_.PublishVisualMsg(link, visual_msg, _req.block);
    _res.status.push_back(true);
    _res.messages.push_back("SetVisualTransparencies: Succcess for visual.");
  }
  _res.success = true;
  _res.status_message = "SetVisualTransparencies: Finished.";
  return true;
}

bool DeepsimGazeboVisualServicesPrivate::SetVisualVisible(
    const deepsim_msgs::SetVisualVisible::Request &_req,
    deepsim_msgs::SetVisualVisible::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);
  gazebo::physics::LinkPtr link =
    boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(_req.link_name));
  if (!link)
  {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_visual_services",
        "SetVisualVisible: link [%s] does not exist", _req.link_name.c_str());
      _res.success = false;
      _res.status_message = "SetVisualVisible: link does not exist";
  }
  gazebo::msgs::Visual visual_msg = link->GetVisualMessage(_req.visual_name);
  visual_msg.set_visible(static_cast<bool>(_req.visible));
  // Publish visual message
  visual_publisher_.PublishVisualMsg(link, visual_msg, _req.block);

  _res.success = true;
  _res.status_message = "SetVisualVisible: Succcess.";
  return true;
}

bool DeepsimGazeboVisualServicesPrivate::SetVisualVisibles(
    const deepsim_msgs::SetVisualVisibles::Request &_req,
    deepsim_msgs::SetVisualVisibles::Response &_res)
{
  std::lock_guard<std::mutex> lock(visuals_lock);
  if ( _req.link_names.size() != _req.visual_names.size() ||
    _req.link_names.size() != _req.visibles.size())
  {
    _res.status_message = "SetVisualTransparencies: link_names, visual_names, visibles must be same size!";
    _res.success = false;
    return true;
  }
  std::vector<std::string>::const_iterator link_name_iter = _req.link_names.begin();
  std::vector<std::string>::const_iterator visual_name_iter = _req.visual_names.begin();
  std::vector<signed char>::const_iterator visible_iter = _req.visibles.begin();

  for (; link_name_iter != _req.link_names.end();
    ++link_name_iter, ++visual_name_iter, ++visible_iter)
  {
    std::string link_name = *link_name_iter;
    std::string visual_name = *visual_name_iter;
    gazebo::physics::LinkPtr link = boost::dynamic_pointer_cast<gazebo::physics::Link>(world_->EntityByName(link_name));
    if (!link)
    {
      ROS_ERROR_NAMED(
        "deepsim_gazebo_visual_services",
        "SetVisualVisibles: link [%s] does not exist", link_name.c_str());
      _res.status.push_back(false);
      _res.messages.push_back("SetVisualVisibles: link does not exist");
      continue;
    }
    gazebo::msgs::Visual visual_msg = link->GetVisualMessage(*visual_name_iter);
    visual_msg.set_visible(static_cast<bool>(*visible_iter));

    // Publish visual message
    visual_publisher_.PublishVisualMsg(link, visual_msg, _req.block);
    _res.status.push_back(true);
    _res.messages.push_back("SetVisualVisibles: Succcess for visual.");
  }
  _res.success = true;
  _res.status_message = "SetVisualVisibles: Finished.";
  return true;
}
}  // namespace deepsim_gazebo_plugin
