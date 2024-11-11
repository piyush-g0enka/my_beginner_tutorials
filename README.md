## ROS2 Assignment 2 - Services/Parameters/Launch files


'beginner_tutorials' package created as part of ROS2 programming assignments.

In the ros_services_logging_launch branch....

#### Dependencies
1. ROS2 Humble
2. C++17
3. Git


#### Build package

```bash
# First clone the package into your ros2_ws

# Run below command from workspace directory
$ colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

#### Run package

##### Launch file
```bash
# change publish_frequency parameter value to change /chatter frequency
$ ros2 launch beginner_tutorials nodes.launch.py publish_frequency:=5.4
```



##### change string service
```bash

# Initially the talker node will output "I'm not set" string in /chatter topic

# To change string to "I'm publishing FLASE" run below service call
$ ros2 service call /string_changer example_interfaces/srv/SetBool "data: false"

# To change string to "I'm publishing TRUE" run below service call
$ ros2 service call /string_changer example_interfaces/srv/SetBool "data: true"
```

#### Cpplint
```bash
# Run below command from package directory
$ cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/talker.cpp src/listener.cpp 
```


#### Clang-tidy
```bash
# Create symbolic link to compile_commands.json in package directory
$ ln -s ~/<path-to-ros2_ws>/build/beginner_tutorials/compile_commands.json ~/<path-to-ros2_ws>/src/beginner_tutorials

# Get clang-tidy output
$ clang-tidy -p compile_commands.json --extra-arg=-std=c++17 src/talker.cpp src/listener.cpp
```
