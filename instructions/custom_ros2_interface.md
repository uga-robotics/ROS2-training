# Creating a custom ROS2 interface

The goal of his tutorial is to create a custom ROS2 Intrface. It is good practice
to use predefined interfaces when you can but there may come a time when you need
to make your own.

## Creating a new package
Let's start by creating a new package in the `tutorial_ws/src` directory called `tutorial_interfaces`
```
ros2 pkg create -build-type ament_cmake tutorial_interfaces
```
Inside the package create two new folders to seperate the `.msg` and `.srv`
files.

## Creating Custom Definitions

In the `msg` directory that you just created create a file named `Num.msg`.
Inside the file you will define its data structure with:
```
int64 num
```

This custom message will transfer a single 64 bit int.

In the `srv` directory create a file called `AddThreeInts.srv` with following
structure:
```
int64 a
int64 b
int64 c
----------
int64 sum
```
This file defines the structure of the request and the response. This service
requires three int names `a`, `b` and `c` and it will return an int called `sum`.

To convert the `.srv` and `.msg` files to C++ code add the following line to the
`CMakeLists.txt`:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
 )

```

Interfaces rely on `rosidl_default_generators` to generate language specific
code and you must declare it as a dependecy in `package.xml`; this adds `rosidl_default_generators` as a build and execution dependecy:
```
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>

```

Build the `tutorial_interfaces` package from the root, `tutorial_ws/`, of the workspace using `colcon`:
```
colcon build --packages-select tutorial_ws

```
Now in another terminal source the `tutorial_ws/` and you will be able to use the interfaces in the ROS package. To insure the
interface is working correctly use the `ros2 interface show command` in the
`dev_ws/src` directory.

```
ros2 interface show tutorial_interfaces/msg/Num
```
The terminal response should be:
```
int64 num
```
Use the same command to show the contents of the `.srv` file:
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```
It should return:
```
int64 a
int64 b
int64 c
---
int64 sum
```

## Testing the Interfaces

You will use the packages that you made in the previous tutorial.

### Testing the `Num.msg` with pub/sub
Inside the Pub/Sub src folder run the following command to create a copy of
`publisher_member_function.cpp` and `subscriber_member_function.cpp` files:

```
cp publisher_member_function.cpp publisher_member_function.cpp.copy &&
cp subscriber_member_function.cpp subscriber_member_function.cpp.copy
```

After making a copy of the files execute the following commands in the Pub/Sub package
src to overwrite `publisher_member_function.cpp` and `subscriber_member_function.cpp`:
```
wget -O publisher_member_function.cpp (URL to file)
wget -O subscriber_member_function.cpp (URL to file)
```
Looking at the overwritten `publisher_member_function.cpp` file you will see the changes needed to use your custom interface. On line five the following `#inlcude` statement has been added:
```
#include "tutorial_interfaces/msg/num.hpp"     // CHANGE
```
This include allow you to use your `Num` message in `publisher_member_function.cpp`. An identical statment can be found on line four of the `subscriber_member_function.cpp`. After including the custom interface, the publisher constructor must be changed to create a publisher that follows the `tutorial_interfaces` `msg` interface:
```C++
  publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);    // CHANGE
```
Once the publisher has the correct type, the `timer_callback()` fucntion needs to be modified to send messages of the correct type:
```C++
auto message = tutorial_interfaces::msg::Num();                               // CHANGE
message.num = this->count_++;                                        // CHANGE
RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);    // CHANGE
publisher_->publish(message);
}
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;         // CHANGE
size_t count_;
```
The message variable is now structured like the `tutorial_interfaces` `Num`. The message's num member, remember the variable names from the `.msg` definition, is incremented everytime the `timer_callback()` is called and the messages is published to the topic.

The Subscriber is also modified to accepet messages like your custom interface. Lines 13 and 14 create a subscription based on the `tutorial_interfaces` `msg`.:
```c++
subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(          // CHANGE
  "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
```

In the `topic_callback` method, the `SharedPtrs` are now part of the `tutorial_interfaces` type and the terminal out retrieves the int form the message.
```c++
void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const       // CHANGE
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);              // CHANGE
}
rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;       // CHANGE
```

After overwriting the files you will need to make the following changes to your
`CMakeLists.txt`:

```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)       # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()

```

The first change tells CMake that it needs to find your `tutorial_interfaces` folder. The second changes adds the folder to the build target dependencies for the `talker`. The third adds the folder to the build target dependencies for the `listener`

In `package.xml` add the following line:
``` xml
<depend>tutorial_interfaces</depend>
```
This line list the `tutorial_interfaces` package as a dependecy of the publisher/ Subscriber package.

After editing the files navigate back to `tutorial_interfaces` and rebuild `cpp_pubsub` using
`colcon`:
```
colcon build --packages-select cpp_pubsub
```
Then open two terminals and run the `talker` and `listener`. Since `Num.msg`
only relays integers, the publisher should only be publishing integer values.

### Testing `AddingThreeInts.srv` with service/client

Navigate to **cpp_srvcli/src** folder and make copies of `add_two_ints_service.cpp`
and `add_two_ints_client.cpp`:
```
cp add_two_ints_service.cpp add_two_ints_service.cpp.copy
cp add_two_ints_client.cpp add_two_ints_client.cpp.copy
```
After making copies of the files overwrite them with:
```
wget -O add_two_ints_service.cpp (insert url to service_custom_interface)
wget -O add_two_ints_client.cpp (insert url to file client_custom_interface)
```

The changes on line two of `service_custom_interface.cpp`:
```
#include "tutorial_interfaces/srv/add_three_ints.hpp"     // CHANGE
```
includes the package in the source file to be used by the client. Lines six and seven change the method signiture to take in a SharedPtr of `tutorial_interfaces`:
```C++
void add(const std::SharedPtr<tutorial_interfaces::srv::AddThreeInts::Request> request,     // CHANGE
          std::SharedPtr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  // CHANGE
{
```
 Lines nine through twelve changes the response to return the sum of the three ints:
 ```C++
 response->sum = request->a + request->b + request->c;                                       // CHANGE
 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",   // CHANGE
               request->a, request->b, request->c);
 ```
  Using the SharedPtr for the request,`request`, you are able to extract the ints sent with the request, i.e`request->a`, after applying operations to the ints you are able to assign the value of the response using its SharedPtr,`response`.

  After changing the `add` function, the `main` function's body must be altered as well. Lines 19 and 24 simply change the message ROS prints to the console at reflect the state of the program:
  ```C++
  std::SharedPtr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");  // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");      // CHANGE
  ```
  Lines 21 and 22 change the service to use the new `tutorial_interfaces` type:
  ```C++
  rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =                 // CHANGE
    node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);     // CHANGE

  ```
Line 19 creates a node `SharedPtr` called `add_three_ints_client:
```
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client"); // CHANGE
```

Line 20 through 21 creates a client node `SharedPtr` that will hold the client based on the `AddThreeInts.srv`:
```
rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =                        // CHANGE
  node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");                  // CHANGE

```
Line 23 creates a request `SharedPtr` based off the `AddThreeInts.srv`:
```C++
auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();
```

The client node in `client_custom_interface.cpp` also had changes made to it. Line 14 ensures the user passes three inputs with the program call one the command line:
```
if (argc != 4) { // CHANGE
  ...
}
```
Line 26 extracts converts the third argument to an int and puts it in the request using the `request` pointer.

After overwriting the files make the following changes the the `CMakeLists.txt`:
```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)                      #CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      #CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```
Much like in the Publisher/Subscriber `CMakeLists` the changes here makes the `tutorial_interfaces` package a dependency of the `server` and `client` build. After modifying the Publisher Subscriber `CMakeLists` add the following line to `package.xml`:
```
<depend>tutorial_interfaces</depend>
```
This again adds the `tutorial_interfaces` package as a dependecy of the Service/ Client package.

After making the changes navigate to `dev_ws` and build the `cpp_svicli` package
with `colcon`:
```
colcon build --package-select (make a package in tutorial_ws)
```

Once the build is complete open two terminals and source `tutorial_ws` before running the following test:
```
ros2 run cpp_srvcli server

ros2 run cpp_srvcli client 2 3 1
```

Custom interfaces can hold more than just int64, ROS has supports a wide variety of primatives and custom class types for a list of supported types read the [following documentation.](https://design.ros2.org/articles/generated_interfaces_cpp.html)
