“Mastering Gazebo Simulator” course teaches how to build robotics simulations using Gazebo and connect it to your ROS environment. This course is a must if you want to learn how to build Gazebo worlds, models or robots. You will learn the essential, follow examples and practice by using the official references of Gazebo.

What You Will Learn

Gazebo GUI
How to build a robot for Gazebo
How to connect gazebo robots to ROS
How to build custom Gazebo worlds
How to write plugins for gazebo worlds and models


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
