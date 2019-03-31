// Copyright 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_DEPTH_CAMERA_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_DEPTH_CAMERA_HPP_

#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <std_msgs/msg/empty.hpp>

namespace gazebo_plugins
{
class GazeboRosDepthCameraPrivate;

//TODO: Add example usage

class GazeboRosDepthCamera : public gazebo::DepthCameraPlugin
{
public:
    /// Constructor
    GazeboRosDepthCamera();

    /// Destructor
    ~GazeboRosDepthCamera();

protected:
    // Documentation inherited
    void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

    void OnNewDepthFrame(const float *_image,
                         unsigned int _width, unsigned int _height,
                         unsigned int _depth, const std::string &_format) override;

    void OnNewRGBPointCloud(const float *_pcd,
                            unsigned int _width, unsigned int _height,
                            unsigned int _depth, const std::string &_format) override;

    void OnNewImageFrame(const unsigned char *_image,
                         unsigned int _width, unsigned int _height,
                         unsigned int _depth, const std::string &_format) override;

private:
    /// Private data pointer
    std::unique_ptr<GazeboRosDepthCameraPrivate> impl_;

};

} // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_DEPTH_CAMERA_HPP_
