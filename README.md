# turtlebot3 velocity controller


Run simulation
`export TURTLEBOT3_MODEL=burger`
`ros2 launch turtlebot3_gazebo empty_world.launch.py`

Run velocity controller
`ros2 run tb-controller velocity-controller `


Created with
`ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs geometry_msgs --node-name velocity-controller --maintainer-name hussein tb-controller`

## [Vicon Bridge](https://github.com/ethz-asl/vicon_bridge)

We pull messages from [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) package, and convert it to ros2 messages. This can be done in a docker. 


## [ROS2/ROS1 bridge](https://github.com/ros2/ros1_bridge)
- The computer which accomplishes the bridging must have both ROS2 and ROS1 installed.
Please ensure that ROS2 = foxy, and ROS1 = noetic 

See following [tutorial](https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS1-ROS2-bridge.html)

Run the following
```
export ROS1_INSTALL_PATH=/opt/ros/noetic
export ROS2_INSTALL_PATH=/opt/ros/foxy/install
```

Then 
```
colcon build --symlink-install --packages-skip ros1_bridge
source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
. dev_ws/local_setup.bash
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

To run, on the bridge machine
```
# Shell A (ROS 1 only):
source ${ROS1_INSTALL_PATH}/setup.bash
roscore
```

In a new shell,
```
# Shell B (ROS 1 + ROS 2):
source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

Run the docker with the vicon bridge as mentioned previously


### PID
We use PID controller from [PID](https://github.com/tekdemo/MiniPID)