# Using Parameters in a class
As we learned in the [core_concepts](core_concepts.md) tutorial, parameters are configuration values for nodes. When creating your own nodes, there may be a time when you need to add parameters that can be set from the launch file. This tutorial will show you how to create those parameters and set them in a launch file.

## Writing a C++ node
Start by sourcing your ros2 directory and navigating to `tutorial_ws/src`, create a new package called `cpp_parameters`:
```shell
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```
Once you have verified that the package has been created navigate into the source directory. You do not have to add `rclcpp` to your `CMakeLists` nor your `package.xml` because you created the package with it listed as a dependency, the `find_package` statements have been added for you. Inside the `src` directory create run the following command:
```
wget -o cpp_parameters_node.cpp (url to file)
```
You should now see a file called `cpp_parameters_node.cpp`, opening the file you should see a class definition for a ROS 2 node. This node class is very similar to the node class seen in the node section of the [core_concepts](core_concepts.md). The main differences are on lines 14 and 20. On line 14 we declare a parameter of type string for the node instance:
```
this->declare_parameter<std::string>("my_parameter", "world");
```
The parameter's name is `"my_parameter"` and its value is `"world"`. On line 20 of the class definition a method of the `Node` class is used to retrieve the value of the parameter `"my_parameter"` and stores it in `parameter_string_`, this variable is defined on line 24 of the class definition:
```
this->get_parameter("my_parameter", parameter_string_);
```
On line 21 you print the value of `my_parameter` to the console. After looking at the code we need to add the it to the `CMakeLists.txt`. Navigate to `cpp_parameters` and open the `CMakeLists.txt` and add the following lines:
```
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
```
rosdep install -i --from-path src --rosdistro foxy -y
```
Then build the packages with `colcon`:
```
colcon build --packages-select cpp_parameters
```
After building the package open a new terminal, do not forget to source the ROS 2 workspace, and source the setup files:
```
. install/setup.bash
```
If you run your ROS 2 package:
```
ros2 run cpp_parameters parameter_node
```
You should see:
```
[INFO] [parameter_node]: Hello world
```
printed to the console. This is the default value of the node's parameters.

## Changing parameters with the Launch File
There may be times when you want to change the parameters of a node while you are launching them, this can be accomplished
