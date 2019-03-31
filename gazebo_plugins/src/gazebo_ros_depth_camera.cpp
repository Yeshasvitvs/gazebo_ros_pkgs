// Copyright 2013 Open Source Robotics Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gazebo_plugins/gazebo_ros_depth_camera.hpp>

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/camera_info.h>
#include <camera_info_manager/camera_info_manager.h>

namespace gazebo_plugins
{
class GazeboRosDepthCameraPrivate
{
public:
    //Pointer to GazeboRos node
    gazebo_ros::Node::SharedPtr ros_node_{nullptr};

    //Camera info manager
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

    //Camera info publisher
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_{nullptr};

    //Image encoding
    std::string type_;

    //Camera name, to be used on ros topics
    std::string camera_name_;

    //Frame name, to be used by TFs
    std::string frame_name_;
};

GazeboRosDepthCamera::GazeboRosDepthCamera()
    : impl_(std::make_unique<GazeboRosDepthCameraPrivate>())
{
}

GazeboRosDepthCamera::~GazeboRosDepthCamera()
{
    //TODO
}

void GazeboRosDepthCamera::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    gazebo::DepthCameraPlugin::Load(_sensor, _sdf);

    //Set camera name
    impl_->camera_name_ = _sdf->Get<std::string>("camera_name", _sensor->Name()).first;

    //Set frame name
    impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

    //Initialize ROS node
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    //Set camera info
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header.frame_id = impl_->frame_name_;
    camera_info_msg.height = this->height;
    camera_info_msg.width = this->width;
    //TODO: Check the distortion model
    camera_info_msg.distortion_model = "plumb_bob";
    camera_info_msg.d.resize(5);
}


}
