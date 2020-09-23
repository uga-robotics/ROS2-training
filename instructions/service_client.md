# Creating a Service and Client
When nodes communicate using services, the node that sends a request for data is called the client node, and the one that responds to the request is the service node. The structure of the request and response is determined by a `.srv` file.

The example used here is a simple integer addition system; one node requests the sum of two integers, and the other responds with the result.

## Prerequisites
Navigate into the `tutorial_ws/src` directory and create a new package called `cpp_srvcli`:
```
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
```
Your terminal will return a message verifying the creation of your package cpp_srvcli and all its necessary files and folders.

The `--dependencies` argument automatically adds the necessary dependecy lines to the `package.xml` and `CMakeLists.txt`. `example_interfaces` is the package that includes the `.srv` file needed to structure your request and response:

```
int64 a
int64 b
---
int64 sum
```
The first two lines define the request data type and variable and the last line is the response.

Because you used the `--dependencies` option during package creation, you don’t have to manually add dependencies to `package.xml` or `CMakeLists.txt`.

As always, though, make sure to add the description, maintainer email and name, and license information to `package.xml`.

```xml
<description>C++ client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

## Anatomy of a Service Node
Inside the `cpp_srvcli/src` directory, make a file called `add_two_ints_server.cpp ` and paste the following code into it:

```C++
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

As in the publisher/subscriber tutorial, the first two `include` statements are your ROS2-specific dependencies, in this case the ROS2 Client Library and our service header file from the `example_interfaces` package.

The `memory` include tells us this particular node utilizes smart pointers, which are in the `memory` header of the C++ Standard Library.

```C++
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>
```
You should notice that we're not encapsulating this node into a class structure, and this is because there is really no need to for such a simple Service. There are no Publisher, Subscriber, or timer objects to keep track of, so a data structure like a class is unnecessary in this instance.

The `add` function adds two integers from the request and gives the sum to the response, while notifying the console of its status using logs.

```C++
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
        request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}
```

The main function accomplishes the following, line by line:

Initializes the ROS2 Client Library:
```C++
rclcpp::init(argc, argv);
```

Creates a node named `add_two_ints_server`:
```C++
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");
```

Creates a service named `add_two_ints` for that node and automatically advertises it over the networks with the `add` function defined above:
```C++
rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
```

Prints a log message when it’s ready:
```C++
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
```

Spins the node, making the service available:
```C++
rclcpp::spin(node);
```

Shuts down the node, when the `spin` function stops, which happens when a node is told to stop via a keyboard interrupt or some other means:
```C++
rclcpp::shutdown();
```

## Anatomy of a Client Node
Inside the `cpp_srvcli/src` directory, create a new file called `add_two_ints_client.cpp` and paste the following code within:

```C++
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

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
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
```

Similar to the service node, the following lines of code create the node and then create the client for that node:
```C++
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
  node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
```

Next, the request is created. Its structure is defined by the `.srv` file included from the `example_interfaces` packagem as we mentioned earlier:
```C++
auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
request->a = atoll(argv[1]);
request->b = atoll(argv[2]);
```

The `while` loop gives the client 1 second to search for service nodes in the network. If it can’t find any, it will continue waiting. If the client is canceled (e.g. by you entering `Ctrl+C` into the terminal), it will return an error log message stating it was interrupted:
```C++
while (!client->wait_for_service(1s)) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    return 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
```

Then the client sends its request using the `async_send_request` function, which will send an `asyncronous` service request to the server. 

In short an `asyncronous` service request can be made from anywhere without running the risk of blocking other ROS2 and non-ROS2 processes, unlike `syncronous` service requests. you can read more about `asyncronous` and `syncronous` requests [here](https://index.ros.org/doc/ros2/Tutorials/Sync-Vs-Async/):

```C++
 auto result = client->async_send_request(request);
```

An `asynchronous` client will immediately return `future`, a value that indicates whether the call and response is finished (not the value of the response itself), after sending a request to a service. The returned `future` may be queried for a response at any time.

Since sending a request doesn’t block anything, a loop can be used to both spin `rclcpp` and check `future` in the same thread, as we can see below.

When the a successful `future` value is finally returned, the client logs either a successful result, or an error:

```C++
// Wait for the result.
if (rclcpp::spin_until_future_complete(node, result) ==
  rclcpp::FutureReturnCode::SUCCESS)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
} else {
  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
}

rclcpp::shutdown();
return 0;
```

Now that we've got both a service and a client, lets update our `CMakeLists.txt` file so we can build and run our new nodes.

## Updating CMakeLists.txt
As you'll recall, since we used the `--dependencies` option when creating our package, all we need to do to update our `CMakeLists.txt` is add our executables and make sure they visible in the workspace.

The `add_executable` macro generates an executable you can run using `ros2 run`. Add the following code block to `CMakeLists.txt` to create two executables named `server` and `client` for our Service and Client respectively:
```
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)
```

To ensure `ros2 run` can find the package add the following lines to the end of the file right before `ament_package()`:

```
install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})
```
After removing some unnecessary boilerplate from the automatically generated file, your final `CMakeLists.txt` should look like this:
 
```
cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp example_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp example_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## Build and Run
It’s good practice to run `rosdep` in the root of your workspace (`tutorial_ws`) to check for missing dependencies before building:
```
rosdep install -i --from-path src --rosdistro foxy -y
```

Navigate to the root of your workspace, `tutorial_ws`, and build your new package:
```
colcon build --packages-select cpp_srvcli
```

Then source the newly generated setup file:
```
. install/setup.bash
```

Now run the service node:
```
ros2 run cpp_srvcli server
```

The terminal should return the following message, and then wait:
```
[INFO] [1600228642.714835791] [rclcpp]: Ready to add two ints.
```

Open another terminal, source the setup files from inside `tutorial_ws` again. Start the client node, followed by any two integers separated by a space:
```
ros2 run cpp_srvcli client 1 2
```

Return to the terminal where your service node is running. You will see that it published log messages when it received the request and the data it received, and the response it sent back:
```
[INFO] [1600228680.492595124] [rclcpp]: Incoming request
a: 1 b: 2
[INFO] [1600228680.492626323] [rclcpp]: sending back response: [3]
```

Enter `Ctrl+C` in the server terminal to stop the node from spinning.

## Next Steps
Now that you know the basics of writing publisher/subscriber and Service/Client Nodes in ROS2, you can move on to creating new message and service types in the [Creating a ROS2 Interface tutorial](custom_ros2_interface.md).