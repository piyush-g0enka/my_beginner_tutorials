## ROS2 Assignment 3 - TF/Unit-Test/Bag-Files


'beginner_tutorials' package created as part of ROS2 programming assignments.

In the ros_tf2_unitTest_bagFiles branch we implement code for TF2 transforms, unit testing in ROS2 and running bag files.

Here, the talker node publishes a string to chatter topic whose publish frequency can be set via command line argument to launch file.
The talker node also contains a service which changes the value of the string being published.
The talker also publishes a static TF transform between /world and /talk frames.

The listener node outputs the string in the chatter with different log levels.

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

#### Run the package

##### Launch file
```bash
# change publish_frequency parameter value to change /chatter frequency
# change record_bag paramter from 'true' to 'false' to not record a bag file for 15 seconds
$ ros2 launch beginner_tutorials nodes.launch.py publish_frequency:=5.4 record_bag:=true

# To record a bag file separately, run below launch file
$ ros2 launch beginner_tutorials record_bag.launch.py record_bag:=true

```


##### Run catch2 test before calling the service

```bash
# Run below command from the ros2_ws directory
# The below test script checks for two conditions:
# 1. Messages are being published on /chatter
# 2. The message data is "I'm not set"

$ ./build/beginner_tutorials/test_talker
```


##### Call a service that changes the string published to the chatter topic
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
$ cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/talker.cpp src/listener.cpp src/test_talker.cpp
```


#### Clang-tidy
```bash
# Create symbolic link to compile_commands.json in package directory
$ ln -s ~/<path-to-ros2_ws>/build/beginner_tutorials/compile_commands.json ~/<path-to-ros2_ws>/src/beginner_tutorials

# Get clang-tidy output - Run from package directory
$ clang-tidy -p compile_commands.json --extra-arg=-std=c++17 src/talker.cpp src/listener.cpp src/test_talker.cpp ; echo "Exit code: $?"
```
