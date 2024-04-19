
# ROS MasterClass Checkpoint 1

This project is for ROS MasterClass from [theconstruct.ai](theconstruct.ai) and for the full Souuce code checkout [my_rb1_robot ](https://github.com/MohammadRobot/my_rb1_robot) repositories

### Task 1   Build RB1 URDF Replica 

![image](/docs/images/rb1.png)



2. Create a new ROS package inside `my_rb1_robot` named `my_rb1_description` with `rospy`, `urdf` and `xacro` as dependencies.


    - `$ catkin_create_pkg my_rb1_description rospy urdf xacro`

3. Inside the package, create a `urdf` folder. Inside this folder, create a file named `my_rb1_robot.urdf`.

    - $ cd my_rb1_description/
    - $ mkdir urdf
    - $ touch urdf/my_rb1_robot.urdf


compute the moments of inertia: 


![image](/docs/images/momentsofinertia.png)
[moments of inertia](https://en.wikipedia.org/wiki/List_of_moments_of_inertia)

Robot base: m =25 h=0.3 r= 0.25
* ixx =  iyy = 0.578125
* izz = 0.78125

wheel : m=1 h=0.05 r=0.025
* ixx = iyy = 3.6459
* izz = 0.0003125

Caster Wheel = m=1 r=0.03
* ixx = iyy = izz 0.00036



8. To visualize the robot

    - $ mkdir launch
    - $ touch launch/display.launch

```xml
<?xml version="1.0"?>
<launch>
  <param name="my_rb1_description" command="cat '$(find my_rb1_description)/urdf/my_rb1_robot.urdf'"/>
  <!-- send fake joint values -->
  <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
  <!-- Combine joint values -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
  <!-- Show in Rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" />
</launch>
```
To view the Robot in Rviz run the following commnand:
- `$ roslaunch my_rb1_description display.launch`
-  Change the frixed framd value in `Fixed Frame` dropdown to `base_footprint`
-  change the `Alpha` value in the `RobotModel` dropdown to `0.5`


To push the code to Github run the followin comands:
- `$ git config --global user.name "Mohammad Alshamsi"`

- `$ git config --global user.email "alshamsi.mohammad@gmail.com"`

- `$ git status`

- `$ git add my_rb1_description/`

- `$ git commit -m "ceate my_rb1_robot.urdf file"`

- `$ git tag task1`


- Create a new repository on GitHub `my_rb1_robot` (don't tick the 'CREATE README" option) https://github.com/MohammadRobot/my_rb1_robot.git

- Generate Personal access tokens

- `git remote add origin https://github.com/MohammadRobot/my_rb1_robot.git`

- `$ git branch -M main`
- `$ git push -u origin main `


## Task 2   Spawn robot in simulation

* `$ cd ~/catkin_ws/src/my_rb1_robot/`
* `$ catkin_create_pkg my_rb1_gazebo `
* `$ mkdir my_rb1_gazebo/launch`
* `$ cp /home/user/simulation_ws/src/warehouse_robot_lab/rb1_base_sim/rb1_base_gazebo/launch/empty_warehouse.launch my_rb1_gazebo/launch/`
* `$ touch my_rb1_gazebo/launch/my_rb1_robot_warehouse.launch`
* `$ mv my_rb1_gazebo/launch/empty_warehouse.launch my_rb1_gazebo/launch/my_rb1_robot_warehouse.launch` 

Add the follow to the my_rb1_robot_warehouse.launch

```xml
<!-- Adding URF file -->  
    <param name="robot_description" command="cat '$(find my_rb1_description)/urdf/my_rb1_robot.urdf'" />
    <arg name="x" default="-0.3"/>
    <arg name="y" default="-1.3"/>
    <arg name="z" default="0.25"/>

    <!-- Spawn the Robot-->
    <node name="mybot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
          args="-urdf -param robot_description -model my_rb1_robot -x $(arg x) -y $(arg y) -z $(arg z)" />
          
    <!-- launch other nodes -->
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"/>
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
```

* `$ roslaunch my_rb1_gazebo my_rb1_robot_warehouse.launch`

To chage the color of the robot in Gazebo add the following to URDF file 
```xml
<!-- Gazebo Tags for Robot Color-->
    <gazebo reference="base_link">
        <material>Gazebo/Black</material>
    </gazebo>
    <gazebo reference="front_caster">
        <material>Gazebo/Grey</material>
    </gazebo>
    <gazebo reference="back_caster">
        <material>Gazebo/Grey</material>
    </gazebo>
    <gazebo reference="left_wheel">
        <material>Gazebo/Red</material>
    </gazebo>
    <gazebo reference="right_wheel">
        <material>Gazebo/Red</material>
    </gazebo>
    <gazebo reference="front_laser">
        <material>Gazebo/Blue</material>
    </gazebo>
```
* 
* `$ git add my_rb1_description/urdf/my_rb1_robot.urdf`
* `$ git add my_rb1_gazebo/`
* `$ git commit -m "Complete Task 2 Spawn robot in simulation"`
* `$ git tag task2`
* `$ git push -u origin task2`
* `$ git push -u origin main`



## Task 3   Add sensor plugins

* Add libgazebo_ros_diff_drive.so plugin to URDF file 

```xml
    <!-- Gazebo Plugin for actuators and sensors -->
    <gazebo>
        <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">
            <alwaysOn>true</alwaysOn>
            <updateRate>20</updateRate>
            <leftJoint>joint_base_left_wheel</leftJoint>
            <rightJoint>joint_base_right_wheel</rightJoint>
            <wheelSeparation>0.4</wheelSeparation>
            <wheelDiameter>0.05</wheelDiameter>
            <torque>5</torque>
            <commandTopic>cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_link</robotBaseFrame>
        </plugin>
    </gazebo>
```
* Add llibgazebo_ros_laser.so plugin to URDF file 
```xml
    <!-- Gazebo Laser Scan -->
    <gazebo reference="front_laser">
        <sensor type="ray" name="head_hokuyo_sensor">
            <pose>0 0 0 0 0 0</pose>
            <visualize>true</visualize>
            <update_rate>20</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>720</samples>
                        <resolution>1</resolution>
                        <min_angle>-1.570796</min_angle>
                        <max_angle>1.570796</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>0.20</min>
                    <max>10.0</max>
                    <resolution>0.01</resolution>
                </range>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </ray>
            <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
                <topicName>/scan</topicName>
                <frameName>sensor_laser</frameName>
            </plugin>
        </sensor>
    </gazebo>
```
* `$ git add my_rb1_description/urdf/my_rb1_robot.urdf`
* `$ git add my_rb1_gazebo/launch/my_rb1_robot_warehouse.launch`
* `$ git commit -m "complete Task 3 Add sensor plugins"`
* `$ git tag task3`
* `$ git push -u origin task3`
* `$ git push -u origin main`

* `$ roslaunch my_rb1_gazebo my_rb1_robot_warehouse.launch`
* to drive the robot run the following commnad


    ```
    $ rostopic pub /cmd_vel geometry_msgs/Twist "linear:

    x: 0.2

    y: 0.0

    z: 0.0

    angular:

    x: 0.0

    y: 0.0

    z: 0.0"
    ```
* to view laser data run `$ rostopic echo /scan`

### Referance 

- For Git commnad check [Git Cheat Sheets](https://training.github.com/downloads/github-git-cheat-sheet/ )



