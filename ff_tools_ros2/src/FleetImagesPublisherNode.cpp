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

#include <chrono>

#include "FleetImagesPublisherNode.hpp"

namespace free_fleet
{
namespace ros2
{

 FleetImagesPublisherNode::FleetImagesPublisherNode() : Node("fleet_images_publisher")
{
  setup_config();

  using namespace std::chrono_literals;
  using std::placeholders::_1;

  publish_image_timer = this->create_wall_timer(
      100ms,std::bind(&FleetImagesPublisherNode::publish_images_callback, this));

  fleet_images_sub = this->create_subscription<FleetImage>(
      this->fleet_image_topic, 10, 
      std::bind(&FleetImagesPublisherNode::fleet_images_callback, this, _1));
}

FleetImagesPublisherNode::~FleetImagesPublisherNode()
{}

void FleetImagesPublisherNode::setup_config()
{
  this->declare_parameter("fleet_image_topic", "fleet_image");
  this->fleet_image_topic =
    this->get_parameter("fleet_image_topic").get_parameter_value().get<std::string>();

  this->declare_parameter("fleet_name", "fleet_name");
  this->fleet_name = 
    this->get_parameter("fleet_name").get_parameter_value().get<std::string>();

  WriteLock robot_images_lock(robot_images_mutex);
  robot_images.clear();
}

void FleetImagesPublisherNode::fleet_images_callback(FleetImage::UniquePtr msg)
{
  WriteLock robot_images_lock(robot_images_mutex);
  for (const rmf_fleet_msgs::msg::RobotImage& ff_ri : msg->robots)
  {
    robot_images[ff_ri.robot_name] = ff_ri.image;
  }
}

void FleetImagesPublisherNode::publish_images_callback()
{
  ReadLock robot_images_lock(robot_images_mutex);
  for (std::pair<std::string, sensor_msgs::msg::Image> image : robot_images)
  {
    auto it = image_publishers.find(image.first);
    if (it == image_publishers.end())
    {
      std::string topic = fleet_name + "/" + image.first;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fleet_image_pub = 
          create_publisher<sensor_msgs::msg::Image>(topic, 10);
      image_publishers[image.first] = fleet_image_pub;
    } else
    {
      it->second->publish(image.second);
    }
  }
}

} // namespace ros2
} // namespace free_fleet