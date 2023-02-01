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

### Instructions for Docker setup
```
sudo docker run -it osrf/ros:noetic-desktop-full
source ros_entrypoint.sh
cd ~/
mkdir -p dev_ws/src
cd ./dev_ws/src
sudo apt install vim git
git clone https://github.com/ethz-asl/vicon_bridge.git
vim vicon_bridge/launch/vicon.launch #change ip to: 192.168.0.254:801 depending on vicon pc ip 
cd ../
catkin_make -DCMAKE_BUILD_TYPE=Release
```

**to save docker**
docker commit dockers-app-1 ros-vicon:version3
docker save -o ros-vicon-v3.tar ros-vicon:version3

**to run vicon bridge**
```
docker exec -it dockers-app-1 bash
source ros_entrypoint.sh
cd ~/dev_ws/
source devel/setup.bash
roslaunch vicon_bridge vicon.launch
```

## [ROS2/ROS1 bridge](https://github.com/ros2/ros1_bridge)
- The computer which accomplishes the bridging must have both ROS2 and ROS1 installed.
Please ensure that ROS2 = foxy, and ROS1 = noetic 

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