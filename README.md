#### build
```bash
# Run below command from workspace directory
$ colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

#### cpplint
```bash
# Run below command from package directory
$ cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/talker.cpp src/listener.cpp 
```


#### clang-tidy
```bash
# Create symbolic link to compile_commands.json in package directory
$ ln -s ~/<path-to-ros2_ws>/build/beginner_tutorials/compile_commands.json ~/<path-to-ros2_ws>/src/beginner_tutorials

# Get clang-tidy output
$ clang-tidy -p compile_commands.json --extra-arg=-std=c++17 src/talker.cpp src/listener.cpp
```
