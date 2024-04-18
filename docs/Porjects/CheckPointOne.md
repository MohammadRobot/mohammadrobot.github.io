

### Task 1   Build RB1 URDF Replica 

![image](/docs/images/rb1.png)

For the full Souuce code checkout [my_rb1_robot ](https://github.com/MohammadRobot/my_rb1_robot) repositories

2. Create a new ROS package inside `my_rb1_robot` named `my_rb1_description` with `rospy`, `urdf` and `xacro` as dependencies.


    - `$ catkin_create_pkg my_rb1_description rospy urdf xacro`

3. Inside the package, create a `urdf` folder. Inside this folder, create a file named `my_rb1_robot.urdf`.

    - $ cd my_rb1_description/
    - $ mkdir urdf
    - $ touch urdf/my_rb1_robot.urdf


compute the moments of inertia: 

![image](/docs/images/momentsofinertia.png)

https://pressbooks.library.upei.ca/statics/chapter/mass-moment-of-inertia/



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

- `$ git checkout task1`

- Create a new repository on GitHub `my_rb1_robot` (don't tick the 'CREATE README" option) https://github.com/MohammadRobot/my_rb1_robot.git

- Generate Personal access tokens

- `git remote add origin https://github.com/MohammadRobot/my_rb1_robot.git`

- `$ git branch -M main`
- `$ git push -u origin main `


### Referance 

- For Git commnad check [Git Cheat Sheets](https://training.github.com/downloads/github-git-cheat-sheet/ )



