# Creating a Service and Client
When Using services, one node, called the client node, sends a request for data to the service node. The structure of the request and the response is determined by a `.srv` file.
## Prerequisites
Make sure you have completed the [create a workspace](create_workspace.md) tutorial.
## Creating a Package
Navigate into the `dev_ws/src` directory and create a new package called `cpp_srvcli`:
```
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
```
The ```--dependencies``` argument automatically adds the necessary dependecy lines to the ```package.xml``` and ```CMakeLists.txt```. ```example_interfaces``` is the package that includes the **.srv** file needed to structure your request and response.
```
int64 a
int64 b
---
int64 sum
```
The first two lines define the request data type and variable and the last line is the response.

## Write the Service and Client Nodes

Inside the `dev_ws/src/cpp_srvcli/src` directory, run the following command to download the practice files:
```
wget -O add_two_ints_service.cpp (insert url to file)
```
You should now have a file the directory called `add_two_ints_server.cpp`.

`add_two_ints_service.cpp` has tow ROS2 dependencies `rclcpp/rclcpp.hpp` and `example_interfaces/srv/add_two_ints.hpp` as well as `memory` from the **C++** standard library. The `add()` method takes in the request adding the two integers, `a` and `b` from the **.srv** file and constructing the response to be sent to the client;
``` c++
void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b; // Sum of request ints are  found
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b); // Message indicating the request was recieved
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum); // Sends response to client
}
```
In the `main` function, a node named "add_two_ints_server" is create along with the service named "add_two_ints" that calls `add` when a client sends a request. The server automatically advertised other the ROS2 network.
```
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

```
Ros2 is initialized and a ready message is displayed in the console:
```
 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
```
Finally the node is spun up and waits for a request from a client. Using the `add_executable` macro to your `CMakeLists.txt` will generate an executable for `ros2 run`. Add the following code to your `CMakeLists.txt`:
```
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
rclcpp example_interfaces)
```
To ensure `ros2 run` can find the package add the following lines to the end of the file right before `ament_package()`:
``` CMakeLists
install(TARGETS
  server
  DESTINATION lib/${PROJECT_NAME})
```
Before testing the server lets create the client.

Inside the `dev_ws/src/cpp_srvcli/src` directory, run the following command to download the practice files:
```
wget -O add_two_ints_client.cpp (insert url to file)
```
You should now have a file the directory called `add_two_ints_client.cpp`. Looking at the code in `add_two_ints_client.cpp` we see that there is only a `main` function and dependencies. The `main` function creates a node called "add_two_ints_client" and creates a client to the "add_two_ints" service:
``` c++
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
```
A request is constructed holding holding ints from the user in the `a` and `b` parameters of the service:
```
auto result = client->async_send_request(request);
```
The client waits for the server to respond with the sum of the two ints it sent if it is successfull it displays the sum to the console:
```
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
```
otherwise it displays an error message to the console:
```
RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
```
After reviewing the code, return to the `CMakeLists.txt` and add the following line:
```
add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp example_interfaces)
```
and `client` to install so `ros2 run` can run it:
```
install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})
```
After adding those lines your `CMakeLists.txt` file should contain the following:
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
Build, run and source the workspace, then start the server:
```
ros2 run cpp_srvcli server
```
The terminal should print a ready message, in a second terminal containing the same workspace run:
```
ros2 run cpp_srvcli client 2 3
```
You should see the terminal print `[INFO] [rclcpp]: Sum: 5` and the server terminal should have:
```
[INFO] [rclcpp]: Incoming request
a: 2 b: 3
[INFO] [rclcpp]: sending back response: [5]

```
