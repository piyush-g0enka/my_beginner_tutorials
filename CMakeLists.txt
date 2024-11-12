cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(talker src/talker.cpp)

add_executable(listener src/listener.cpp)

ament_target_dependencies(talker rclcpp std_msgs example_interfaces)
ament_target_dependencies(listener rclcpp std_msgs)


install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# Install include directories
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
