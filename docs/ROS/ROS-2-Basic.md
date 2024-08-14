# ROS 2

ROS 2 (Robot Operating System 2) is an
open source software development kit for robotics
applications. The purpose of ROS 2 is to offer a
standard software platform to developers across
industries that will carry them from research
and prototyping through to deployment and
production. ROS 2 builds on the success of ROS 1,
which is used today in myriad robotics applications
around the world

## Simulation 

craete an accont on : https://www.theconstruct.ai

Open the Rosjects: https://app.theconstruct.ai/l/6400c71b


To conrol the robot by Keyboard run the following command 

    ros2 run teleop_twist_keyboard teleop_twist_keyboard 


## ROS 2 package

ROS2 uses packages to organize its programs. Think of a package as all the files that a specific ROS program contains; all its CPP files, Python files, configuration files, compilation files, launch files, and parameter files. Also, organizing your ROS2 programs in packages makes sharing them with other developers/users much easier.

Every package will have the following structure of files and folders:

- launch folder: Contains launch files
- src folder: Contains source files (CPP, Python)
- CMakeLists.txt: List of Cmake rules for compilation
- package.xml: Package metadata and dependencies


### Create a Package 

Go to ROS2 Workspace:

    cd ~/ros2_ws/src

Create Package by runing this command:

    ros2 pkg create topic_publisher_pkg --build-type ament_cmake --dependencies rclcpp std_msgs

To compile the package:

    cd ~/ros2_ws/
    colcon build

Source setup.bash from the install folder so that ROS can find the packages in the workspace

    source install/setup.bash

To list ROS2 Packages 

    ros2 pkg list


### Launch File

To run ROS 2 program you can use launch file the by runing the following command: 

    ros2 launch <package_name> <launch_file>

To create launch file 
    
    cd ~/ros2_ws/src/topic_publisher_pkg
    mkdir launch
    touch simple_topic_publisher.launch.py
    chmod +x simple_topic_publisher.launch.py


Sample code of launch file

```py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_publisher_pkg',
            executable='simple_publisher_node',
            output='screen'),
    ])

```

### Source File (src)

Create a C++ file in the src directory of topic_publisher_pkg

    cd ~/ros2_ws/src/topic_publisher_pkg/src
    touh simple_topic_publisher.cpp

Copy the following code: 

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher()
  : Node("simple_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimplePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_;
    count_++;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}

```


### CMakeLists.txt file  

add the following code to  CMakeLists.txt befor `ament_package()` line

```cmake
add_executable(simple_publisher_node src/simple_topic_publisher.cpp)
ament_target_dependencies(simple_publisher_node rclcpp std_msgs)

install(TARGETS
   simple_publisher_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ament_package()
 
```

### Compiling the package 

To compile the package run the following command:

    cd ~/ros2_ws/
    colcon build
    source install/setup.bash



##  Visualize Data

### RVIZ 
To open rviz run the following commmad 

    rviz2

### rqt_graph 
To open rqt_graph  run the following commmad 

    rqt_graph 




## Referaces 

- [theconstruct.ai](https://www.theconstruct.ai/)
- [docs.ros.org](https://docs.ros.org)