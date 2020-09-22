# Creating a ROS2 Interface
In previous tutorials you utilized message and service interfaces to learn about topics, services, and simple publisher/subscriber and service/client nodes. The interfaces you used were predefined in those cases.

While it’s good practice to use predefined interface definitions, you will probably need to define your own messages and services sometimes as well. This tutorial will introduce you to the simplest method of creating custom interface definitions.

## prerequisites
For this tutorial you will be creating custom `.msg` and `.srv` files in their own package, and then utilizing them in a separate package. Both packages should be in the same workspace.

Since we will use the pub/sub and service/client packages created in earlier tutorials, make sure you are in the same workspace as those packages (`tutorial_ws/src`), and then run the following command to create a new package:
```
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```
`tutorial_interfaces` is the name of the new package. Note that it is a CMake package; there currently isn’t a way to generate a `.msg` or `.srv` file in a pure Python package. You can create a custom interface in a CMake package, and then use it in a Python node, which will be covered in the last section.

It is good practice to keep `.msg` and `.srv` files in their own directories within a package. Create the directories in `tutorial_ws/src/tutorial_interfaces`:
```
mkdir msg

mkdir srv
```

## Creating Definitions

Creating message and service definitions is a relatively simple process in ROS2, simply define the structure in a `.msg` or `.srv` file, set up your workspace to generate/build said files, and integrate them into your ROS2 Nodes.

Interfaces can currently only be defined in CMake packages. It is possible, however, to have Python libraries and nodes in CMake packages (using `ament_cmake_python`), so you could define interfaces and Python nodes together in one package. We’ll use a CMake package and C++ nodes here for the sake of simplicity.

This tutorial will focus on the `msg` and `srv` interface types, but the steps here are applicable to `action` types as well.

### msg Definition
In the `msg` directory that you just created create a file named `Num.msg`.
Inside the file you will define its data structure with:
```
int64 num
```
As we've seen before, the structure of a `.msg` file is simply the contents of the message, structured either with [ROS2 primitive types](https://design.ros2.org/articles/interface_definition.html) or other message types.

This message transfers a single 64-bit integer called `num`.

### srv Definition
Back in the tutorial_interfaces/srv directory you just created, make a new file called AddThreeInts.srv with the following request and response structure:
```
int64 a
int64 b
int64 c
----------
int64 sum
```

This is your custom service that requests three 64-bit integers named `a`, `b`, and `c`, and responds with a 64-bit integer called `sum`.

### Generating the Interfaces
To convert the interfaces you defined into language-specific code (like C++ and Python) so that they can be used in those languages, add the following lines to `CMakeLists.txt`:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
 )
```
For C++, these lines tell CMake to generate a `.hpp` or "header" file with the message or service definition defined inside of it, allowing it to be `#include`'ed into any node on the machine.

After interface generation for the package has been set up, we need to add a few things to our `package.xml`. Because the interfaces rely on `rosidl_default_generators` for generating language-specific code, you need to declare a dependency on it. Add the following lines to `package.xml`:
```xml
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

After that, to use the messages, we need to build the package. In the root of your workspace (`~/tutorial_ws`), run the following command:
```
colcon build --packages-select tutorial_interfaces
```

Now in another terminal source the `tutorial_ws/` and you will be able to use the interfaces in the ROS2 package. To insure the interface is working correctly use the `ros2 interface show command`:
```
ros2 interface show tutorial_interfaces/msg/Num
```
The response should be:
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
For this step you can use the packages you created in previous tutorials. A few simple modifications to the nodes, `CMakeLists` and `package` files will allow you to use your new interfaces.

### Testing `Num.msg` with pub/sub
Inside the Pub/Sub src folder run the following command to create a copy of
`publisher_member_function.cpp` and `subscriber_member_function.cpp` files:
```
cp publisher_member_function.cpp publisher_member_function_num.cpp && cp subscriber_member_function.cpp subscriber_member_function_num.cpp
```

This will copy our existing code into a new file with `_num` appended to the name, as we'll be using the `num` interface.

After that, make the following changes to the new publisher and subscriber files

Publisher:
```C++
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::Num();
    message.num = this->count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

Subscriber:
```C++
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<tutorial_interfaces::msg::Num>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);
  }
  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

`CMakeLists.txt`:
```
find_package(tutorial_interfaces REQUIRED)                             # ADD

add_executable(talker_num src/publisher_member_function_num.cpp)       # ADD
ament_target_dependencies(talker_num rclcpp tutorial_interfaces)       # ADD

add_executable(listener_num src/subscriber_member_function_num.cpp)    # ADD
ament_target_dependencies(listener_num rclcpp tutorial_interfaces)     # ADD

install(TARGETS                                                        
  talker
  listener
  talker_num                                                           # ADD
  listener_num                                                         # ADD
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

`package.xml`:
```xml
<depend>tutorial_interfaces</depend>
```

After making the above edits and saving all the changes, navigate to the root of the workspace (`tutorial_ws/`) and build the package. Note: you should make sure the `tutorial_interfaces` package is built before attempting to build the `cpp_pubsub` package:
```
colcon build --packages-select cpp_pubsub
```

Now, if we source our workspace setup file (`tutorial_ws/install/setup.*`), we can run our newly modified executables, and verify our `Num` message type is working:
```
ros2 run cpp_pubsub talker_num
```
```
ros2 run cpp_pubsub listener_num
```

Since `Num.msg` relays only an integer, the talker should only be publishing integer values, as opposed to the string it published previously:
```
[INFO] [minimal_publisher]: Publishing: '0'
[INFO] [minimal_publisher]: Publishing: '1'
[INFO] [minimal_publisher]: Publishing: '2'
```

### Testing `AddThreeInts.srv` with service/client
With some slight modifications to the service/client package created in a previous tutorial, you can see `AddThreeInts.srv` in action. Since you’ll be changing the original two integer request srv to a three integer request srv, the output will be slightly different.

So, same as before, we'll create copies of our original files, Navigate to the `cpp_srvcli/src` folder and make copies of `add_two_ints_service.cpp` and `add_two_ints_client.cpp`, naming them `add_three_ints_service.cpp` and `add_three_ints_client.cpp`:
```
cp add_two_ints_server.cpp add_three_ints_server.cpp && cp add_two_ints_client.cpp add_three_ints_client.cpp
```

then replace the contents of `add_three_ints_service.cpp` with:
```C++
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"

#include <memory>

void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,
          std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)
{
  response->sum = request->a + request->b + request->c;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",
                request->a, request->b, request->c);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");

  rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =
    node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

and the contents of `add_three_ints_client.cpp` with:
```C++
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_three_ints.hpp"        // CHANGE

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) { // CHANGE
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");
  rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =
    node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");

  auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);
  request->c = atoll(argv[3]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");
  }

  rclcpp::shutdown();
  return 0;
}
```

Then make the following changes to `CMakeLists.txt`:
```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                            # ADD

add_executable(server_three src/add_three_ints_server.cpp)            # ADD
ament_target_dependencies(server_three rclcpp tutorial_interfaces)    # ADD

add_executable(client_three src/add_three_ints_client.cpp)            # ADD
ament_target_dependencies(client_three rclcpp tutorial_interfaces)    # ADD

install(TARGETS
  server
  client
  server_three                                                        # ADD
  client_three                                                        # ADD
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

and add this to `package.xml`
```xml
<depend>tutorial_interfaces</depend>
```

After making the above edits and saving all the changes, navigate to the root of the workspace (`tutorial_ws/`) and build the package. Note: you should make sure the `tutorial_interfaces` package is built before attempting to build the `cpp_srvcli` package:
```
colcon build --packages-select cpp_srvcli
```

Now, if we source our workspace setup file (`tutorial_ws/install/setup.*`), we can run our newly modified executables, and verify our `AddThreeInts` service type is working:
```
ros2 run cpp_srvcli server_three
```
```
ros2 run cpp_srvcli client_three 1 2 3
```

## Next Steps
Now that you've got a decent grasp on making your own interfaces, [let's move on to using parameters in a class](using_parameters.md).