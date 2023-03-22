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

#ifndef FF_TOOLS_ROS2__SRC__FLEETIMAGESNODECONFIG_HPP
#define FF_TOOLS_ROS2__SRC__FLEETIMAGESNODECONFIG_HPP

#include <string>

namespace free_fleet
{
namespace ros2
{

struct FleetImagesNodeConfig
{
  
  std::string fleet_name = "fleet_name";

  std::string fleet_state_topic = "fleet_state";
  std::string fleet_image_topic = "fleet_image";

  int dds_domain = 42;

  double update_state_frequency = 10.0;
  double publish_state_frequency = 10.0;

  double update_image_frequency = 10.0;
  double publish_image_frequency = 10.0;

};

} // namespace ros2
} // namespace free_fleet

#endif // FF_TOOLS_ROS2__SRC__FLEETIMAGESNODECONFIG_HPP
