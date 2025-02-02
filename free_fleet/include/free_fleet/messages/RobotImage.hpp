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

#ifndef FREE_FLEET__INCLUDE__FREE_FLEET__MESSAGES__ROBOTIMAGE_HPP
#define FREE_FLEET__INCLUDE__FREE_FLEET__MESSAGES__ROBOTIMAGE_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "Header.hpp"
#include "ImagePixel.hpp"


namespace free_fleet {
namespace messages {

struct RobotImage
{
  std::string fleet_name;
  std::string robot_name;
  Header image_header;
  uint32_t height;
  uint32_t width;
  std::string encoding;
  bool is_bigendian;
  uint32_t step;
  std::vector<ImagePixel> data;
};

} // namespace messages
} // namespace free_fleet

#endif // FREE_FLEET__INCLUDE__FREE_FLEET__MESSAGES__ROBOTIMAGE_HPP
