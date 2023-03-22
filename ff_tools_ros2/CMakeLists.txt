cmake_minimum_required(VERSION 3.5.0)

project(ff_tools_ros2 LANGUAGES C CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
  find_package(builtin_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(rmf_fleet_msgs REQUIRED)

  add_executable(ff_tools_ros2
    src/main.cpp
    src/FleetImagesPublisherNode.cpp
  )
  target_link_libraries(ff_tools_ros2
  )
  target_include_directories(ff_tools_ros2
    PRIVATE
  )
  ament_target_dependencies(ff_tools_ros2
    rclcpp
    std_msgs
    sensor_msgs
    rmf_fleet_msgs
  )

  
  install(
    TARGETS ff_tools_ros2
    RUNTIME DESTINATION lib/ff_tools_ros2
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  ament_package()

else()
  message("ament_cmake not found so skipping this ROS2 package")
endif()

# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------


