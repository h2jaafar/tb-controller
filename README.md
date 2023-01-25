# turtlebot3 velocity controller


Run simulation
`ros2 launch turtlebot3_gazebo empty_world.launch.py`

Run velocity controller
`ros2 run tb-controller velocity-controller `


Created with
`ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs geometry_msgs --node-name velocity-controller --maintainer-name hussein tb-controller`

## Vicon Bridge

We pull messages from [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) package, and convert it to ros2 messages. This can be done in a docker. 

