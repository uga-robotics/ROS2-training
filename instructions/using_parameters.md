# Using Parameters in a class
As we learned in the [core_concepts](core_concepts.md) tutorial, parameters are configuration values for nodes. When creating your own nodes, there may be a time when you need to add parameters that can be set from a `launch` file. This tutorial will show you how to create those parameters and how to set them in a launch file.

## Prerequisites
Before we get started, navigate to the `src` of your tutorial workspace `tutorial_ws/src`, and create a new package called `cpp_parameters`:
```
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```
Once you have verified that the package has been created, navigate into its `src` directory. 

You do not have to add `rclcpp` to your `CMakeLists.txt` nor your `package.xml` as when you created the package with it listed as a dependency, the `find_package` statements were added for you. However you still need to fill out the `<description>`, `<maintainer>` and `<license>` tags in the `package.xml`, failure to do this will result in wierd `colcon` build errors. 

Inside the `src` directory create a file named `cpp_parameters_node.cpp` and paste the following into it:
```C++
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("parameter_node")
    {
      this->declare_parameter<std::string>("my_parameter", "world");
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParametersClass::respond, this));
    }
    void respond()
    {
      this->get_parameter("my_parameter", parameter_string_);
      RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
    }
  private:
    std::string parameter_string_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}
```

In the next section, we'll go through the code line-by-line to explain how Parameters can be read, written, and used in ROS2 nodes.

## Writing a C++ Node With Parameters
The `#include` statements at the top are the package dependencies. 

Generally, It is good practice to include the C++ library headers you'll be using to write your node, although the common ones such as `<functional>`, `<memory>`, etc. have likely already been included by the client library, so they are not strictly necessary, although strongly recommended (as long as header files are defined correctly, including a header twice will not cause issues).

```C++
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;
```

The next piece of code creates the class and the constructor, naming the node `parameter_node`. The first line of this constructor creates our parameter. Our parameter has the name `my_parameter` and is assigned the default value world. Next, `timer_` is initialized, which causes the `respond` function (defined next) to be executed once a second (1000 ms):
```C++
class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("parameter_node")
    {
      this->declare_parameter<std::string>("my_parameter", "world");
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParametersClass::respond, this));
    }
```

The first line of our respond function gets the parameter `my_parameter` from the node, and stores it in the private class variable `parameter_string_`. The `RCLCPP_INFO` function ensures the message is logged.
```C++
void respond()
{
  this->get_parameter("my_parameter", parameter_string_);
  RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
}
```

Last is the declaration of the private `timer_` and `parameter_string_` class variables.
```C++
private:
  std::string parameter_string_;
  rclcpp::TimerBase::SharedPtr timer_;
```

Following our `ParametersClass` is our `main` function. Here ROS2 is initialized, and a `ParameterClass` object is created and passed into `rclcpp::spin`, which starts processing data from the node.
```C++
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}
```

## Building and running the node
Before building your workspace, we'll need to modify our `CMakeLists.txt`. Open up the `cpp_parameters/CMakeLists.txt` file, and below the dependency `find_package(rclcpp REQUIRED)` add the following lines:
```
add_executable(parameter_node src/cpp_parameters_node.cpp)
ament_target_dependencies(parameter_node rclcpp)

install(TARGETS
  parameter_node
  DESTINATION lib/${PROJECT_NAME}
)
```

After modifying `CMakeLists.txt` go to the root of the directory `tutorial_ws` and run the following command to check for missing dependencies:
```
rosdep install -i --from-path src --rosdistro foxy -y
```
Then build the package with `colcon`:
```
colcon build --packages-select cpp_parameters
```
After building the package open a new terminal, do not forget to source the ROS2 workspace, and source the setup files:
```
. install/setup.*
```
If you run the `parameters_node` Node:
```
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
```
ros2 run cpp_parameters parameter_node
```
With the node running open a new terminal source `ros` and the `tutorial_ws` and run the following command to list the node's parameters:
```
ros2 param list
```
You should see:
```
/parameter_node:
    my_parameter
    use_sim_time
```
Use the following command to change the parameter while the node is running:
```
ros2 param set /parameter_node my_parameter earth
```
You should see the console output change from `Hello world` to `Hello earth`. You can use the `param set` command to change the parameters of any running node.

## Changing parameters with the Launch File
There may be times when you want to change node parameters while you are launching them, this can be accomplished by creating nodes in a launch file and specifying the parameters. Create a new directory called `launch` in the `cpp_parameters` package; in the new directory, create a new file called `cpp_parameters_launch.py` and paste the following lines into it:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="parameter_node",
            name="custom_parameter_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
```
Inside the launch description, a Node is created running `parameter_node` from the `cpp_parameters` package:
``` py
Node(
    package="cpp_parameters",
    executable="parameter_node",
```
The node is named `custom_parameter_node` and to ensure the output is printed to the console, the output variable is set to `screen` and `emulate_tty` is set to `True`:
```py
    name="custom_parameter_node",
    output="screen",
    emulate_tty=True,
```

The variable is `parameter` gives developers the ability to set the parameters of nodes using key value pairs and curly brackets (as mentioned in the launch file tutorial, LaunchDescription is in JSON format):
```python
parameters=[
    {"my_parameter": "earth"}
]
```
`parameters` is a list type which means you can list multiple parameters in one statement by seperating them with commas:
```python
parameters=[
    {parameter_1 : value},
    {parameter_2 : value},
    {parameter_3 : value},
    {parameter_X : value}
]
```
After creating the launch file, you will need to edit `CMakeLists.txt`. Add the following lines:
```
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

Then rebuild the package, make sure you are in `tutorial_ws/`:
```
colcon build --packages-select cpp_parameters
```
Open an new terminal windows and source ROS2, then source the setup for the `tutorial_ws` workspace:
```
. install/setup.*
```

After sourcing the workspace launch the node using the `launch` file:
```
ros2 launch cpp_parameters cpp_parameters_launch.py
```

You should see the following message displayed in the terminal:
```
[parameter_node-1] [INFO] [custom_parameter_node]: Hello earth
```

## Next Steps
This concludes the UGA Robotics Club introductory tutorials for ROS2! Although we've actually already covered a lot of the topic in them, you should be able to work through the [Intermediate and Advanced ROS2 tutorials](https://index.ros.org/doc/ros2/Tutorials/) on your own, with more advanced topics like Robot Localization and Navigation Systems, Gazebo/TF/URDF, etc. being covered in workshops in the future.
