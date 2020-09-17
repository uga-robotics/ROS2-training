# Creating a Publisher and Subscriber
Nodes are executable processes that communicate over the ROS graph. In this tutorial, the nodes will pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

When writing ROS2 nodes, C++ will be our main language for anything that needs to be even remotely "performant" or "realish-time" and Python will be used as a "scripting" language for quick tasks/nodes that don't necessarily need to perform tons of operations.

## Prerequisites
We'll set up our workspace for this tutorial by creating a new ROS2 package in the `tutorial_ws` workspace called `cpp_pubsub`, remember to put packages in the `src` directory of the workspace:
```
ros2 pkg create --build-type ament_cmake cpp_pubsub
```
Your terminal will return a message verifying the creation of your package cpp_pubsub and all its necessary files and folders.

Navigate into the `tutorial_ws/src/cpp_pubsub/src` directory. Recall that this is the directory in any CMake package where the source files containing executables belong.

## Anatomy of a Publisher Node
To demonstrate how to write a publisher node, we'll download some example code using the following command:

```
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp
```
After you've got the file downloaded, open it up in your preffered text editor, we'll go through it line by line, and explain what each section is doing.

The top of the code includes the standard C++ headers you will be including when making any ROS2 executable. The top block here contains includes from the C++ standard library which ROS2 depends on. The bottom block has the `rclcpp/rclcpp.hpp` include which allows you to use the most common pieces of the ROS 2 system, and `std_msgs/msg/string.hpp`, which includes the built-in message type you will use to publish data. You can see a full list of standard ROS2 message types [here]().

The last bit of this code block has a namespace delcaration, which is simply to make available all of the chronographic literals from the `chrono` standard library. ROS2 uses these literals when creating timers, as we'll see later.

These lines represent the node’s dependencies. Recall that dependencies have to be added to package.xml and CMakeLists.txt, which you’ll do in the next section.

```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
```

The next line creates the node class `MinimalPublisher` by inheriting from `rclcpp::Node`. Every `this` in the code is referring to the node.

```C++
class MinimalPublisher : public rclcpp::Node
```

The public constructor names the node `minimal_publisher` and initializes `count_` to 0. Inside the constructor, the publisher is initialized with the `String` message type, the topic name `topic`, and the required queue size to limit messages in the event of a backup. Next, `timer_` is initialized with the parameters, which causes the `timer_callback` function to be executed twice a second. This is where our `using namespace std::chrono_literals;` declarations comes in handy, because we can easily set the timer to 500ms (half a second) between callbacks.

Callbacks are essential to the way ROS2 operates, it is essentially a function that gets called when a particular event happens, such as when a timer goes off, or when a new message appears on a topic (as we'll see in the subscriber section).

```C++
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }
```

The `timer_callback` function is where the message data is set and the messages are actually published, the function is called whenever the timer callback is triggered, which we instantiated in the previous segment. We then instantiate a new String message, and set it's `data` field to `Hello, world!` with a counter appended. The `RCLCPP_INFO` macro ensures every published message is printed to the console. Finally, we publish the message.

```C++
Private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
```

Last in the class definition is the delcaration of our timer, publisher, and counter fields. The publisher is created via a template, which takes in the ROS2 message type you want to publish.

You might have noticied the weird `SharedPtr` after the publisher and timer declarations, this is a built-in smart pointer definition which the ROS2 Client Library defines. If you're not familiar with C++14 or smart pointers, they are essentially managed pointers, which don't need to be manually destroyed by the programmer like a regular pointer. You can read more about smart pointers and their various types [here](https://docs.microsoft.com/en-us/cpp/cpp/smart-pointers-modern-cpp?view=vs-2019).

```C++
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
size_t count_;
```

Following the `MinimalPublisher` class is main, where the node actually executes. `rclcpp::init` initializes ROS2, and `rclcpp::spin` starts processing data from the node, including callbacks from the timer. Again, a smart pointer is used to hold the `MinimalPublisher` object.

```C++
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## Adding Dependencies to CMakeLists.txt and package.xml
Navigate one level back to the `tutorial_ws/src/cpp_pubsub` directory, where the `CMakeLists.txt` and `package.xml` files have been created for you.

### Package.xml
Open `package.xml` with your text editor.

As mentioned in the previous tutorial, make sure to fill in the `<description>`, `<maintainer>` and `<license>` tags:

```xml
<description>Examples of minimal publisher/subscriber using rclcpp</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

Add a new line after the `ament_cmake` buildtool dependency and paste the following runtime dependencies corresponding to your node’s include statements:

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

This declares the package needs `rclcpp` and `std_msgs` when its code is executed. Make sure to save the file.

### CMakeLists.txt
In the same directory, open `CMakeLists.txt` with your text editor.

Below the existing dependency `find_package(ament_cmake REQUIRED)`, add the lines:

```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

Which will tell the build system to find the `rclcpp` and `std_msgs` packages before building, and to abort the build if they can't be found (as they are `REQUIRED`).

After that, add the executable and name it `talker` so you can run your node using ros2 run:

```
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
```

Finally, add the following install(TARGETS…) section so `ros2 run` can find your executable when you build and source the workspace:

```
install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

From here, you could build the package, source the workspace, and run the node, but it will be a lot more interesting once we have the subscriber up and running.

## Anatomy of a Subscriber Node
Navigate back into the `cpp_pubsub/src` directory and run the following command to get the example subscriber file:

```
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp
```

Open the `subscriber_member_function.cpp` file with your text editor.

The subscriber node’s code is nearly identical to the publisher’s. Now the node is named `minimal_subscriber`, and the constructor uses the node’s `create_subscription` class to execute the callback.

There is no timer because the subscriber simply responds whenever data is published to the `topic` topic.

```C++
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
```

Recall from the topic tutorial that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.

The `topic_callback` function receives the string message data published over the topic, and simply writes it to the console using the `RCLCPP_INFO` macro.

The only field declaration in this class is the subscription.

```C++
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
```

The main function is exactly the same, except now it spins the `MinimalSubscriber` node. For the publisher node, spinning meant starting the timer, but for the subscriber it simply means preparing to receive messages whenever they come.

Since this node has the same dependencies as the publisher node, there’s nothing new to add to `package.xml`.

### Adding to the CMakeLists.txt for the Subscriber Node
Reopen `CMakeLists.txt` and add the executable and target for the subscriber node below the publisher’s entries.

```
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

## Build and Run the New Package
You likely already have the `rclpp` and `std_msgs` packages installed as part of your ROS2 system. It’s good practice to run `rosdep` in the root of your workspace (tutorial_ws) to check for missing dependencies before building:

```
rosdep install -i --from-path src --rosdistro foxy -y
```

Still in the root of your workspace, build your new package:

```
colcon build --packages-select cpp_pubsub
```

Now source your new setup files

```
. install/setup.bash

```

Now you can run the talker with:

```
ros2 run cpp_pubsub talker
```

and in another terminal, source the setup files again, and run the listener:

```
ros2 run cpp_pubsub listener
```

The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

# Next Steps
Now that you're aquainted with the basics of writing nodes in ROS2, it's time to take a look at an example [Service and Client](service_client.md)
