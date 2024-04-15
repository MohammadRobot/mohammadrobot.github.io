

 YDLidar-SDK https://github.com/YDLIDAR/YDLidar-SDK

YDLIDAR ROS Driver https://github.com/YDLIDAR/ydlidar_ros_driver 



update X2.launch file 

cd ~/ydlidar_ws/src/ydlidar_ros_driver/launch/X2.launch

    <param name="angle_min"    type="double" value="-180" />
    <param name="angle_max"    type="double" value="180" />

    args="0.0 0.0 0.2 0.0 0.0 0.0 /laser_frame /base_footprint  40" />

To follwoing:

    <param name="angle_min"    type="double" value="5" />
    <param name="angle_max"    type="double" value="175" />

    args="0.0 0.0 0.2 1.57 0.0 0.0 /laser_frame /base_footprint  40" />


update ydlidar_ws/src/ydlidar_ros_driver/launch/lidar.rviz

    Fixed Frame: base_footprint


run the code

    roslaunch ydlidar_ros_driver lidar_view.launch 

    roslaunch ydlidar_ros_driver X2.launch 