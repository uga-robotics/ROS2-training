# Using Parameters in a class
As we learned in the [core_concepts](core_concepts.md) tutorial, parameters are configuration values for nodes. When creating your own nodes, there may be a time when you need to add parameters that can be set from the launch file. This tutorial will show you how to create those parameters and set them in a launch file.

## Writing a C++ node
Start by sourcing your ros2 directory and navigating to `tutorial_ws/src`, create a new package called `cpp_parameters`:
```shell
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```
Once you have verified that the package has been created navigate into the source directory. You do not have to add `rclcpp` to your `CMakeLists` nor your `package.xml` because you created the package with it listed as a dependency, the `find_package` statements have been added for you. Inside the `src` directory create run the following command:
```shell
wget -o cpp_parameters_node.cpp (url to file)
```
You should now see a file called `cpp_parameters_node.cpp`, opening the file you should see a class definition for a ROS 2 node. This node class is very similar to the node class seen in the node section of the [core_concepts](core_concepts.md). The main differences are on lines 14 and 20. On line 14 we declare a parameter of type string for the node instance:
```C++
this->declare_parameter<std::string>("my_parameter", "world");
```
The parameter's name is `"my_parameter"` and its value is `"world"`. On line 20 of the class definition a method of the `Node` class is used to retrieve the value of the parameter `"my_parameter"` and stores it in `parameter_string_`, this variable is defined on line 24 of the class definition:
```C++
this->get_parameter("my_parameter", parameter_string_);
```
On line 21 you print the value of `my_parameter` to the console. After looking at the code we need to add it to the `CMakeLists.txt`. Navigate to `cpp_parameters` and open the `CMakeLists.txt` and add the following lines:
``` CMakeLists
add_executable(parameter_node src/cpp_parameters_node.cpp)
ament_target_dependencies(parameter_node rclcpp)

install(TARGETS
  parameter_node
  DESTINATION lib/${PROJECT_NAME}
)
```
These lines will make the `cpp_parameters_node.cpp` to compile as a part of `parameter_node` and sets `rclcpp` as a dependency, we said earlier that package `find_package` statement was added during package creation but it is not added as a build dependency automatically.

## Building and running the node

After modifying `CMakeLists.txt` go to the root of the directory `tutorial_ws` and run the following command to check for missing dependencies:
```shell
rosdep install -i --from-path src --rosdistro foxy -y
```
Then build the packages with `colcon`:
```shell
colcon build --packages-select cpp_parameters
```
After building the package open a new terminal, do not forget to source the ROS 2 workspace, and source the setup files:
```shell
. install/setup.bash
```
If you run your ROS 2 package:
```shell
ros2 run cpp_parameters parameter_node
```
You should see:
```
[INFO] [parameter_node]: Hello world
```
printed to the console. This is the default value of the node's parameters.

## Changing parameters with the console
Before continuing, make sure you have read the parameters section of the [common_tools](common_tools.md) tutorial. You will be applying what you learned to the node's parameters.

Make sure the node is running, if it is not launch the node with the following command:
```shell
ros2 run cpp_parameters parameter_node
```
With the node running open a new terminal source `ros` and the `tutorial_ws` and run the following command to list the node's parameters:
```shell
ros2 param list
```
You should see:
```
/parameter_node:
    my_parameter
    use_sim_time
```
Use the following command to change the parameter while the node is running:
```shell
ros2 param set /parameter_node my_parameter earth
```
You should see the console output change from `Hello world` to `Hello earth`. You can use the `param set` command to change the parameters of any running node.
## Changing parameters with the Launch File
There may be times when you want to change node parameters while you are launching them, this can be accomplished by creating nodes in a launch file and specifying the parameters. Create a new directory called launch in the `cpp_parameters` package; in the new directory, run the following command:
 ```shell
 wget -o cpp_parameters_launch.py (url to file)
 ```
You should see a new file in the directory called `cpp_parameters_launch.py`. Taking a look inside, you see a the structure of the launch file is very similar to the one [common_tools](common_tools.md). Inside the launch description, a Node is created running `parameter_node` from the `cpp_parameters` package:
``` py
Node(
    package="cpp_parameters",
    executable="parameter_node",
```
The node is named `custom_parameter_node` and to ensure the output is the console, the output variable is set to `screen` and make `emulate_tty` True:

```py
    name="custom_parameter_node",
    output="screen",
    emulate_tty=True,
```
The new variable is `parameter`you are able to set the parameters of nodes using key value pairs and curly brackets:
```
parameters=[
    {"my_parameter": "earth"}
]
```
`Parameters` is a list type which means you can list multiple parameters in one statement by seperating them with commas:
``` python
parameters=[
    {parameter_1 : value},
    {parameter_2 : value},
    {parameter_3 : value},
    {parameter_X : value}
]
```
After creating the launch file, you will need to edit `CMakeLists`. Add the following lines:
```CMake
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```
Then rebuild the package, make sure you are in `tutorial_ws`:
```
colcon build --packages-select cpp_parameters
```
Open an new terminal windows and source ros2, then source the setup if for the `cpp_parameters` package while in `tutorial_ws`:
```
. install/setup.bash
```
After sourcing the workspace launch the node using the launch file:
```
ros2 launch cpp_parameters cpp_parameters_launch.py
```
You should see the following message displayed in the terminal:
```
[parameter_node-1] [INFO] [custom_parameter_node]: Hello earth
```
