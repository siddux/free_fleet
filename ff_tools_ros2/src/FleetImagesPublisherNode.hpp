/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef FF_TOOLS_ROS2__SRC__FLEETIMAGESPUBLISHERNODE_HPP
#define FF_TOOLS_ROS2__SRC__FLEETIMAGESPUBLISHERNODE_HPP

#include <mutex>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>


#include <rmf_fleet_msgs/msg/fleet_image.hpp>

namespace free_fleet
{
namespace ros2
{

class FleetImagesPublisherNode : public rclcpp::Node
{
public:

  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;
  using FleetImage = rmf_fleet_msgs::msg::FleetImage;

  FleetImagesPublisherNode();

  ~FleetImagesPublisherNode();

private:
  // --------------------------------------------------------------------------

  std::string fleet_image_topic;

  std::string fleet_name;

  void setup_config();

  // --------------------------------------------------------------------------

  std::mutex robot_images_mutex;

  std::unordered_map<std::string, sensor_msgs::msg::Image>
      robot_images;

  rclcpp::Subscription<FleetImage>::SharedPtr
      fleet_images_sub;

  void fleet_images_callback(FleetImage::UniquePtr msg);

  // --------------------------------------------------------------------------

  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
      image_publishers;

  rclcpp::TimerBase::SharedPtr publish_image_timer;

  void publish_images_callback();

  // --------------------------------------------------------------------------

};

} // namespace ros2
} // namespace free_fleet

#endif // FF_TOOLS_ROS2__SRC__FLEETIMAGESPUBLISHERNODE_HPP