

## ROS package

ROS uses packages to organize its programs. You can think of a package as all the files that a specific ROS program contains; all its cpp files, python files, configuration files, compilation files, launch files, and parameters files.

All those files in the package are organized with the following structure:
* launch folder: Contains launch files
* src folder: Source files (cpp, python)
* CMakeLists.txt: List of cmake rules for compilation
* package.xml: Package information and dependencies

To go to any ROS package, ROS gives you a command named roscd. When typing:

`$ roscd <package_name>`



All launch files are contained within a <launch> tag. Inside that tag, you can see a <node> tag, where we specify the following parameters:



### Create a package

When we want to create packages, we need to work in a very specific ROS workspace, which is known as the catkin workspace. The catkin workspace is the directory in your hard disk where your own ROS packages must reside in order to be usable by ROS. Usually, the catkin workspace directory is called catkin_ws.
`$ catkin_create_pkg <package_name> <package_dependecies>`

* `$ rospack list`: Gives you a list with all of the packages in your ROS system.
* `$ rospack list | grep <package_name>`: Filters, from all of the packages located in the ROS system,the package named package_name.
* `$ roscd package_name`: Takes you to the location in the Hard Drive of the package, named package_name

###  Compile a package

* `$ cd ~/catkin_ws`
* `$ catkin_make`

After compiling, it's also very important to source your workspace. This will make sure that ROS will always get the latest changes done in your workspace.

* `$ source devel/setup.bash`

## ROS program
### Program file
Create the programe file in `src` directory 
* `$ touch src/<program name>.cpp`

### Launch file
Create launch direcotory and **launch** file whicg contain the following: 
* pkg="package_name" # Name of the package that contains the code of the ROS program to execute
* type="cpp_executable_name" # Name of the cpp executable file that we want to execute
* name="node_name" # Name of the ROS node that will launch our C++ file
* output="type_of_output" # Through which channel you will print the output of the program

To create the launch file run the followin command 
* `$ mkdir launch`
* `$ touch launch/<launch name>.launch`

### CMakeLists File
Modify the CMakeLists.txt file in order to generate an executable from the C++ file you have just created.

In the Build section of your CMakeLists.txt file, add the following lines:

```
add_executable(simple src/<program name>.cpp)
add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple
   ${catkin_LIBRARIES}
 )
```



## rostopic
1. rostopic list

    Display a list of current topics.

        $ rostopic list

1. rostopic echo

    Display messages published to a topic.

        $ rostopic echo /topic_name

For more information visit: http://wiki.ros.org/rostopic 


## Publisher and Subscriber

For more information visit: http://wiki.ros.org/roscpp_tutorials/Tutorials/WritingPublisherSubscriber