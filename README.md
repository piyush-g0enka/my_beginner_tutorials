#### build
```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

#### cpplint
```bash
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/talker.cpp src/listener.cpp > cpplint_output.txt
```


#### clang-tidy
```bash
ln -s /home/robotics/courses/chang/ros2_tutorials/ros2_tutorials_ws/build/beginner_tutorials/compile_commands.json /home/robotics/courses/chang/ros2_tutorials/ros2_tutorials_ws/src/beginner_tutorials

clang-tidy -p compile_commands.json --extra-arg=-std=c++17 src/talker.cpp src/listener.cpp
```
