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

#include <deepsim_gazebo_plugin/deepsim_gazebo_visual_publisher.hpp>

#include <memory>

namespace deepsim_gazebo_plugin
{
class DeepsimGazeboVisualPublisherPrivate
{
public:
  /// \brief The call back queue for ros services
  ros::CallbackQueue & ros_callback_queue_;

  /// \brief Count the number of connections to visual publisher.
  int pub_visuals_connection_count_;

  /// \brief Contructor that binds the callback queue
  /// to protect variables accessed on callbacks.
  /// \param[in] _ros_callback_queue ros::CallbackQueue call back queue for gz services
  explicit DeepsimGazeboVisualPublisherPrivate(
    ros::CallbackQueue & _ros_callback_queue_)
  : ros_callback_queue_(_ros_callback_queue_),
    pub_visuals_connection_count_(0) {}

  /// \brief Callback to update the visual message in the link.
  void UpdateVisualMsg(
    gazebo::physics::LinkPtr &link,
    const gazebo::msgs::Visual &visual_msg);

  /// \brief Callback for a subscriber connecting to Visuals ros topic.
  void OnVisualsConnect();

  /// \brief Callback for a subscriber disconnecting from Visuals ros topic.
  void OnVisualsDisconnect();

  /// \brief Callbacks to publish the visuals to a ros topic.
  void PublishVisuals();

  /// \brief Gazebo node for communication.
  gazebo::transport::NodePtr gz_node_;

  /// \brief pointer to gazebo's ~/visual topic publisher
  gazebo::transport::PublisherPtr gz_visual_pub_;

  /// \brief ROS Pubisher to publish visuals
  ros::Publisher pub_visuals_;

  /// \brief Publish Visuals Gazebo Event.
  gazebo::event::ConnectionPtr pub_visuals_event_;

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// \brief ROS node for communication, managed by gazebo_ros.
  boost::shared_ptr<ros::NodeHandle> ros_node_;

  /// \brief mutex lock for updating publisher connection count
  std::mutex lock_;
};

DeepsimGazeboVisualPublisher::DeepsimGazeboVisualPublisher(
  ros::CallbackQueue & _ros_callback_queue)
: impl_(std::make_unique<DeepsimGazeboVisualPublisherPrivate>(
  _ros_callback_queue))
{
}

DeepsimGazeboVisualPublisher::~DeepsimGazeboVisualPublisher()
{
  // Disconnect if there are subscribers on exit
  if (impl_->pub_visuals_connection_count_ > 0)
  {
    impl_->pub_visuals_event_.reset();
    ROS_DEBUG_STREAM_NAMED(
      "deepsim_gazebo_visual_publisher",
      "Disconnected World Updates");
  }
}

void DeepsimGazeboVisualPublisher::CreatePublisher(
  const gazebo::transport::NodePtr _gz_node,
  const boost::shared_ptr<ros::NodeHandle> _ros_node,
  const gazebo::physics::WorldPtr _world)
{
  impl_->gz_node_ = _gz_node;
  impl_->ros_node_ = _ros_node;
  impl_->world_ = _world;

  // create gazebo visual publisher for updating visuals
  impl_->gz_visual_pub_ = impl_->gz_node_->Advertise<gazebo::msgs::Visual>("~/visual");

  // publish complete visuals in world frame
  ros::AdvertiseOptions pub_visuals_ao =
    ros::AdvertiseOptions::create<deepsim_msgs::Visuals>(
        "visuals", 1,  // queue size is 1 since we are only interested in the latest status
        boost::bind(&DeepsimGazeboVisualPublisherPrivate::OnVisualsConnect, impl_.get()),
        boost::bind(&DeepsimGazeboVisualPublisherPrivate::OnVisualsDisconnect, impl_.get()),
        ros::VoidPtr(), &impl_->ros_callback_queue_);
  impl_->pub_visuals_ = impl_->ros_node_->advertise(pub_visuals_ao);
  impl_->pub_visuals_connection_count_ = 0;  // Reset connection counts

  ROS_INFO_NAMED(
    "deepsim_gazebo_visual_publisher",
    "[deepsim_gazebo_visual_publisher] Visual Publishers Created.");
  ROS_INFO_NAMED(
      "deepsim_gazebo_visual_pubisher",
      "[pub_visuals_connection_count_] count=[%d]",
      impl_->pub_visuals_connection_count_);
}

void DeepsimGazeboVisualPublisher::PublishVisualMsg(
  gazebo::physics::LinkPtr &_link,
  const gazebo::msgs::Visual &_visual_msg,
  const bool block)
{
  gazebo::msgs::Visual new_visual_msg = _visual_msg;
  new_visual_msg.set_name(_link->GetScopedName());
  // publish visual message
  impl_->gz_visual_pub_->Publish(new_visual_msg, block);
  if (block)
  {
    while (impl_->gz_visual_pub_->GetOutgoingCount() > 0)
    {
        impl_->gz_visual_pub_->SendMessage();
    }
  }
  impl_->UpdateVisualMsg(_link, _visual_msg);
}

void DeepsimGazeboVisualPublisherPrivate::OnVisualsConnect()
{
  std::lock_guard<std::mutex> lock(lock_);
  pub_visuals_connection_count_++;
  // Connect on first subscriber
  if (pub_visuals_connection_count_ == 1)
  {
    pub_visuals_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DeepsimGazeboVisualPublisherPrivate::PublishVisuals, this));
    ROS_INFO_NAMED(
      "deepsim_gazebo_visual_pubisher",
      "[deepsim_gazebo_visual_pubisher] Connect on first subscriber.");
  }
}

void DeepsimGazeboVisualPublisherPrivate::OnVisualsDisconnect()
{
  std::lock_guard<std::mutex> lock(lock_);
  pub_visuals_connection_count_--;
  // Disconnect with no subscribers
  if (pub_visuals_connection_count_ <= 0)
  {
    pub_visuals_event_.reset();
    if (pub_visuals_connection_count_ < 0)  // should not be possible
      ROS_ERROR_NAMED(
        "deepsim_gazebo_visual_publisher",
        "One too many disconnect to pub_visuals? count=[%d]",
        pub_visuals_connection_count_);
  }
}

void DeepsimGazeboVisualPublisherPrivate::PublishVisuals()
{
  deepsim_msgs::Visuals visuals;
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
            visuals.link_name.push_back(visual.link_name);
            visuals.visual_name.push_back(visual.visual_name);
            visuals.ambient.push_back(visual.ambient);
            visuals.diffuse.push_back(visual.diffuse);
            visuals.specular.push_back(visual.specular);
            visuals.emissive.push_back(visual.emissive);
            visuals.transparency.push_back(visual.transparency);
            visuals.visible.push_back(visual.visible);
            visuals.geometry_type.push_back(visual.geometry_type);
            visuals.mesh_geom_filename.push_back(visual.mesh_geom_filename);
            visuals.mesh_geom_scale.push_back(visual.mesh_geom_scale);
            visuals.pose.push_back(visual.pose);
        }
      }
    }
  }
  pub_visuals_.publish(visuals);
}

void DeepsimGazeboVisualPublisherPrivate::UpdateVisualMsg(
  gazebo::physics::LinkPtr &_link,
  const gazebo::msgs::Visual &_visual_msg)
{
    gazebo::physics::Link::Visuals_M &visuals = _link->visuals;

    gazebo::physics::Link::Visuals_M::iterator iter;
    for (iter = visuals.begin(); iter != visuals.end(); ++iter)
    {
        if (iter->second.name() == _visual_msg.name())
        {
            iter->second = _visual_msg;
            break;
        }
    }
}
}  // namespace deepsim_gazebo_plugin
