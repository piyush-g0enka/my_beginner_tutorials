cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Catch2 REQUIRED)

add_executable(talker src/talker.cpp)

add_executable(listener src/listener.cpp)

add_executable(test_talker src/test_talker.cpp)

ament_target_dependencies(talker rclcpp std_msgs example_interfaces geometry_msgs tf2_ros)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(test_talker rclcpp std_msgs)

target_link_libraries(test_talker Catch2::Catch2WithMain)
# Register the test
ament_add_test(
  test_talker
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND test_talker
)

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
