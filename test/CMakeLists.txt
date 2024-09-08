cmake_minimum_required(VERSION 3.5)
project(ros2_persistent_parameter_server_test VERSION 1.0.1)

# Set Release build if no build type was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Build type for the build. Possible values are: Debug, Release, RelWithDebInfo, MinSizeRel"
      FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable additional warnings and warnings as errors
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)

add_executable(client
  src/test.cpp
  src/persist_parameter_client.cpp
)
ament_target_dependencies(client
  rclcpp
  rclcpp_components
  std_msgs
  rcutils
)

# add client sample a
add_executable(client_a_node
  src/client_a.cpp
)
ament_target_dependencies(client_a_node
  rclcpp
)
# add client sample b
add_executable(client_b_node
  src/client_b.cpp
)
ament_target_dependencies(client_b_node
  rclcpp
)

target_link_libraries(client
  rclcpp::rclcpp
  rcutils::rcutils
)

target_include_directories(
  client
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  client
  client_a_node
  client_b_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
