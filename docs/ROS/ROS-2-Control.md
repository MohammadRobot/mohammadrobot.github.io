# ros2_control for ROS 2 Robots

This page is a practical introduction to `ros2_control` for mobile robots, robot arms, and simulation projects.

The goal is to understand what `ros2_control` does, how the pieces fit together, and what a minimal setup looks like before you move to real hardware.

Official references:

- [ros2_control documentation](https://control.ros.org/)
- [Controller Manager docs](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html)
- [diff_drive_controller docs](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [joint_trajectory_controller docs](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)

![ros2_control architecture overview](../images/ros2-control/ros2-control-architecture.svg){ width="1100" }

High-level view: ROS 2 commands go into controllers, `controller_manager` arbitrates interfaces, the hardware layer talks to the robot, and feedback comes back as joint states, odometry, and TF.

## 1. What `ros2_control` is

`ros2_control` is the standard ROS 2 control framework used to connect:

- controllers
- hardware drivers
- robot description
- command topics and actions
- joint feedback

It gives you a clean structure instead of writing custom motor code directly in random ROS nodes.

Typical data flow:

```text
cmd_vel / trajectory / command topic
        ->
controller
        ->
controller_manager
        ->
hardware interface
        ->
motors / actuators / simulator
        ->
joint state feedback
        ->
controllers + robot_state_publisher + RViz
```

## 2. Main building blocks

### Controller Manager

`controller_manager` is the core runtime node.

It:

- loads controllers
- manages controller lifecycle
- reads hardware states
- updates controllers
- writes commands back to hardware

### Hardware component

The hardware layer exposes command and state interfaces for your robot.

The main hardware types are:

- `system` for a whole robot or multi-joint device
- `actuator` for a single actuator
- `sensor` for sensors that expose state interfaces

### Controllers and broadcasters

Controllers consume interfaces and generate commands.

Broadcasters publish state without commanding hardware.

Common examples:

| Package | Use |
| --- | --- |
| `joint_state_broadcaster` | publish joint states |
| `diff_drive_controller` | two-wheel mobile bases |
| `joint_trajectory_controller` | robot arms and multi-joint motion |
| `forward_command_controller` | simple direct command testing |

## 3. When to use `ros2_control`

Use it when you want:

- a clean control architecture for a real robot
- the same control pattern in simulation and on hardware
- standard controllers instead of custom one-off motor nodes
- proper interface ownership between multiple controllers
- easier debugging of command and feedback paths

If your robot is growing beyond one quick demo node, `ros2_control` is usually the right direction.

## 4. Install the main packages

Install the framework and standard controllers:

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers
```

If you are using Gazebo Sim, install the integration package too:

```bash
sudo apt install ros-$ROS_DISTRO-gz-ros2-control
```

## 5. Minimal `ros2_control` block in URDF

All joints used by `ros2_control` must already exist in your robot URDF.

This minimal example uses mock hardware so you can test the control pipeline before connecting real motors:

```xml
<ros2_control name="DriveBaseSystem" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
  </hardware>

  <joint name="left_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-20.0</param>
      <param name="max">20.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_wheel_joint">
    <command_interface name="velocity">
      <param name="min">-20.0</param>
      <param name="max">20.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

Why this matters:

- the controller reads and writes named interfaces
- the joint names must match the URDF exactly
- the command and state interfaces must match what the controller expects

For a real robot, replace `mock_components/GenericSystem` with your own hardware plugin.

## 6. Minimal controller configuration

For a differential-drive robot, a small controller YAML usually starts like this:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    wheel_separation: 0.36
    wheel_radius: 0.075

    publish_rate: 50.0
    base_frame_id: base_link
    odom_frame_id: odom
    use_stamped_vel: true
```

Important idea:

- `joint_state_broadcaster` publishes feedback
- `diff_drive_controller` commands the wheels

For a robot arm, the usual next controller is `joint_trajectory_controller`, not `diff_drive_controller`.

## 7. Typical bring-up sequence

The normal startup pattern is:

1. Start `robot_state_publisher` with the robot URDF or xacro.
2. Start `ros2_control_node` from `controller_manager`.
3. Spawn the controllers you need.

Typical controller spawn commands:

```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner diff_drive_controller
```

If you use launch files, the `spawner` helper is the standard way to load and activate controllers during startup.

![ros2_control differential drive bring-up](../images/ros2-control/ros2-control-diff-drive-bringup.svg){ width="1100" }

This is the minimum structure that usually works well for a mobile base: robot description plus controller YAML into `ros2_control_node`, then spawn `joint_state_broadcaster` and `diff_drive_controller`.

## 8. Useful CLI commands

These commands are the fastest way to see whether your control stack is actually alive:

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 control list_controller_types
```

What you want to see:

- hardware component loaded and active
- command interfaces available
- controllers loaded
- broadcasters and controllers in the expected lifecycle state

## 9. Quick test for a differential drive robot

If your controller name is `diff_drive_controller`, publish a velocity command like this:

```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.2}, angular: {z: 0.4}}}" -r 10
```

Then check:

- `/joint_states`
- `/odom`
- TF between `odom` and `base_link`
- wheel joint motion in RViz or simulation

## 10. Common mistakes

These are the failures people hit most often:

- Joint names in YAML do not match the URDF.
- The controller expects `velocity` or `position` interfaces, but the hardware exports something else.
- `joint_state_broadcaster` was never started.
- The robot description is valid URDF, but the `<ros2_control>` block is missing or incomplete.
- Wheel separation or wheel radius values are wrong, so odometry looks broken even though the controller is running.
- Hardware loads, but the component is not active yet.

When debugging, inspect interfaces first. Most `ros2_control` problems are interface-matching problems.

![ros2_control debugging flow](../images/ros2-control/ros2-control-debug-flow.svg){ width="1100" }

Debug in that order. If step `2` or step `3` is wrong, the controller will usually never behave correctly no matter how much you tune it.

## 11. Real robot vs simulation

The useful part of `ros2_control` is that the controller layer can stay mostly the same while the hardware plugin changes.

Typical progression:

- start with `mock_components/GenericSystem`
- move to `gz_ros2_control` in simulation
- replace the hardware plugin with your real motor driver interface

That lets you test the command pipeline before touching the real robot.

## 12. Design advice

Good practice for robotics projects:

- keep the hardware plugin thin and hardware-specific
- keep robot geometry and joints correct in URDF first
- start with one broadcaster and one controller
- prove the interfaces with CLI tools before adding navigation or MoveIt
- tune wheel radius, wheel separation, and transmission assumptions early

`ros2_control` works best when the robot description, interfaces, and controller expectations are all explicit and consistent.

## 13. What to learn next

After the first working setup, the next topics worth learning are:

- writing a custom hardware interface
- using `joint_trajectory_controller` for manipulators
- `gz_ros2_control` for simulation
- controller switching
- debugging controller startup and interface claims

If you can bring up `joint_state_broadcaster`, activate one controller, and verify the exported interfaces from the CLI, you already understand the most important part of `ros2_control`.
