“Mastering Gazebo Simulator” course teaches how to build robotics simulations using Gazebo and connect it to your ROS environment. This course is a must if you want to learn how to build Gazebo worlds, models or robots. You will learn the essential, follow examples and practice by using the official references of Gazebo.

What You Will Learn

Gazebo GUI
How to build a robot for Gazebo
How to connect gazebo robots to ROS
How to build custom Gazebo worlds
How to write plugins for gazebo worlds and models


## Git 

GitHub

$ git remote add origin https://github.com/MohammadRobot/robot_racetrack.git
$ git branch -M main
$ git push -u origin main

Username: MohammadRobot
Password: Personal access tokens generated from https://github.com/settings/tokens


rosbag record -O race-track-rDBk5zAY.bag odm --duration 60


**Unit 3:   Connect to ROS**

This unit presents concepts around creating a new world in Gazebo. You will learn to create new worlds from scratch with different models, ground, animated objects, and actors. In addition, you will learn how to use plugins and how to control worlds programmatically.

### Exercise 3.1 

`$ rostopic echo /gazebo/model_states`

```
$ rostopic pub /gazebo/set_model_state gazebo_msgs/ModelState "model_name: 'unit_box'
pose:
  position:
    x: 10.0
    y: 20.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0
reference_frame: ''"
```

## Gazebo services -
Gazebo starts with several services available. However, most of them are complex, for example, the /gazebo/spaw_urdf_model, which is internally used when you spawn a robot in a launch file. Therefore, it is not practical to use it from the CLI.


 /gazebo/spaw_urdf_model

/gazebo/pause_physics       pause  the simulation
/gazebo/unpause_physics     unpause the simulation
/gazebo/reset_simulator     will put all objects back in their initial places.
/gazebo/reset_world         does the same, except it will also reset the simulation time

`$ rosservice call /gazebo/reset_world`


`roslaunch robot_description spawn.launch`

`roslaunch robot_description spawn.launch x:=6 model_name:=second_model`



### 3.2   ROS Plugin - Robot differential driver

You have the robot ready to move! Your next step is to connect it to ROS is to make it possible to move the robot by it publishing to a ROS topic. There is a plugin for it, the gazebo_ros_diff_drive, and it can be included in the URDF model using the Gazebo tag. This is how it goes:



publisher.py 

```
# Do not skip line 2 
#!/usr/bin/env python 
  
import rospy 
# the following line depends upon the 
# type of message you are trying to publish 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
  
def callback(data):
    print('X: %s, Y: %s' % (data.pose.pose.position.x, data.pose.pose.position.y))
    #print("callback")


def subpub(): 
    # Subscribe to topic 
    sub = rospy.Subscriber('/odom', Odometry, callback)  
    # define the actions the publisher will make 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
    # initialize the publishing node 
    rospy.init_node('gazebo', anonymous=True) 
      
    # define how many times per second 
    # will the data be published 
    # let's say 10 times/second or 10Hz 
    rate = rospy.Rate(10) 
    # to keep publishing as long as the core is running 
    while not rospy.is_shutdown(): 
        data = Twist()
        data.linear.x= 0.5
        data.angular.z= 0.2
          
        # you could simultaneously display the data 
        # on the terminal and to the log file 
        #rospy.loginfo(data) 
          
        # publish the data to the topic using publish() 
        pub.publish(data) 
          
        # keep a buffer based on the rate defined earlier 
        rate.sleep() 
  
  
if __name__ == '__main__': 
    # it is good practice to maintain 
    # a 'try'-'except' clause 
    try: 
        subpub() 
    except rospy.ROSInterruptException: 
        pass
```




It was shown the commonly used plugins for basic robots. You can check all plugins available here:

    http://wiki.ros.org/gazebo_ros_pkgs
    https://github.com/ros-simulation/gazebo_ros_pkgs


## Unit 4:   Build a World


### Creating Gazebo Models

package and create a folder to contain your models:

  Execute

`cd ~/catkin_ws/src/studying_gazebo`

`mkdir -p models/box_model`

`cd models/box_model`

`touch model.config`

`touch model.sdf`

Do not use materials or textures in this first moment. Instead, copy and paste the content below to the model.config file. Change author and other information accordingly:

  Copy and paste
```
<?xml version="1.0"?>

<model>
  <name>Box Model</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>Author's name</name>
    <email>no-author@mail.com</email>
  </author>

  <description>
    A simple box model
  </description>
</model>
```
And the following to the model.sdf file:

 
```
<?xml version='1.0'?>
<sdf version="1.4">
  <model name="box_model">
    <pose>0 0 0.5 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.083</iyy>
          <iyz>0.0</iyz>
          <izz>0.083</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```


## Unit 5:   Write Gazebo Plugins

World Plugin


- Exercise 5.2.1. -

Create a new plugin called world_plugin.

  1. Create the file world_plugin.cc
    
    $ cd ~/catkin_ws
    $ catkin_create_pkg world_plugin gazebo gazebo_ros gazebo_plugins roscpp
    $ touch ~/catkin_ws/src/world_plugin/src/my_gazebo_world_plugin.cc
  ```
  #include <gazebo/gazebo.hh>
  #include <gazebo/physics/physics.hh>
  namespace gazebo {
  class MyGazeboPlugin : public WorldPlugin {
  public:
    MyGazeboPlugin() : WorldPlugin() { printf("Plugin constructor method!\n"); }

  public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
      // set a node
      transport::NodePtr node(new transport::Node());
      node->Init(_world->Name());

      // set publisher
      transport::PublisherPtr publisher =
          node->Advertise<msgs::Factory>("~/factory");

      // create msg obj
      msgs::Factory msg;

      // model to use
      msg.set_sdf_filename("model://jersey_barrier");

      // setup new position
      msgs::Set(msg.mutable_pose(),
                ignition::math::Pose3d(ignition::math::Vector3d(5, 4, 0),
                                      ignition::math::Quaterniond(0, 0, 0)));
      // Send the message
      publisher->Publish(msg);
    }
  };

  // Register plugin
  GZ_REGISTER_WORLD_PLUGIN(MyGazeboPlugin)
  } // namespace gazebo
  ```


    $ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib

update CMakeLists.txt
```
```
cmake_minimum_required(VERSION 3.0.2)
project(world_plugin)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(my_gazebo_world_plugin SHARED src/my_gazebo_world_plugin.cc)
target_link_libraries(my_gazebo_world_plugin ${GAZEBO_LIBRARIES})
```
```

    $ cd ~/catkin_ws
    $ catkin_make
    $ source devel/setup.bash



    $ mkdir -p ~/catkin_ws/src/world_plugin/launch
    $ touch ~/catkin_ws/src/world_plugin/launch/gazebo.launch


gazebo.launch      
``
  <launch>
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
      <arg name="world_name" value="$(find world_plugin)/worlds/gazebo.world" />
      <arg name="paused" value="false"/>
      <arg name="use_sim_time" value="true"/>
      <arg name="gui" value="true"/>
      <arg name="headless" value="false"/>
      <arg name="debug" value="false"/>
    </include>
  </launch>
```


    $ mkdir -p ~/catkin_ws/src/world_plugin/worlds
    $ touch ~/catkin_ws/src/world_plugin/worlds/gazebo.world


gazebo.world
```  ```
  <?xml version="1.0"?>
  <sdf version="1.4">
    <world name="default">
      <plugin name="my_gazebo_world_plugin" filename="libmy_gazebo_world_plugin.so"/>
    </world>
  </sdf>

```

  1. Add the necessary instructions in CMakeLists.txt

```
cmake_minimum_required(VERSION 3.0.2)
project(world_plugins)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(my_gazebo_world_plugin SHARED src/my_gazebo_world_plugin.cc)
target_link_libraries(my_gazebo_world_plugin ${GAZEBO_LIBRARIES})

```

    $ roslaunch world_plugin gazebo.launch




### - Exercise 5.3.1 -


touch ~/catkin_ws/src/writing_plugins/worlds/model.world

touch ~/catkin_ws/src/writing_plugins/src/a_model_plugin.cc




