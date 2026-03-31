一、环境

cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs


二、底盘单独启动

cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dashgo_driver_ros2 dashgo_driver_node --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200


三、雷达单独启动

cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dashgo_lidar_ros2 rplidar_s2.launch.py


四、底盘 + 雷达 + 相机

cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_robot.launch.py


五、底盘 + 雷达

cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_robot.launch.py start_d435:=false


六、真机导航

cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py


七、真机导航（不启相机）

cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_d435:=false


八、真机导航（不启 RViz）

cd /home/xu/project/dashgo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/roslogs
ros2 launch dashgo_driver_ros2 dashgo_nav_real.launch.py start_d435:=false start_nav_rviz:=false


九、常用检查

ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /points_raw --once
ros2 topic echo /combined_grid --once
ros2 topic echo /path --once
