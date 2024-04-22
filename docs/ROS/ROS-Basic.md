
# ROS
ROS is basically the framework that allows us to do all that we showed along this chapter. It provides the background to manage all these processes and communications between them... and much, much more!! In this tutorial you've just scratched the surface of ROS, the basic concepts. ROS is an extremely powerful tool. If you dive into our courses you'll learn much more about ROS and you'll find yourself able to do almost anything with your robots!

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

* `$ catkin_make`
##  Roscore

The roscore is the main process that manages all of the ROS system. You always need to have a roscore running in order to work with ROS. The command that launches a roscore is:
* `$ roscore`


## ROS Nodes 

ROS nodes are basically programs made in ROS. The ROS command to see what nodes are actually running in a computer is:

* `$ rosnode list`

* `$ rosndoe info /<node_name>`


## Parameter Server
 Parameter Server is a dictionary that ROS uses to store parameters. These parameters can be used by nodes at runtime and are normally used for static data, such as configuration parameters.

 * `$ rosparam list`

 To get a value of a particular parameter, you can type:
 * `$rosparam get <parameter_name>`


And to set a value to a parameter, you can type:
* `$ rosparam set <parameter_name> <value>`

## Environment Variables
ROS uses a set of Linux system environment variables in order to work properly. You can check these variables by typing:

* `$ export | grep ROS`

The most important variables are the ROS_MASTER_URI and the ROS_PACKAGE_PATH.

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




## Referaces 

[theconstruct.ai](https://www.theconstruct.ai/)

ROS Packages: http://wiki.ros.org/Packages

Ros Nodes: http://wiki.ros.org/Nodes

Parameter Server: http://wiki.ros.org/Parameter%20Server

Roscore: http://wiki.ros.org/roscore

ROS Environment Variables: http://wiki.ros.org/ROS/EnvironmentVariables
