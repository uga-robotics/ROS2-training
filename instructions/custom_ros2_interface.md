# Creating a custom ROS2 interface

The goal of his tutorial is to create a custom ROS2 Intrface. It is good proactice
to use predefined interface when you can but you may need to create your own.

## Creating a new package

Create a new package in the `dev_ws/src` directory called tutorial_interface
```
ros2 pkg create -build-type ament_cmake tutorial_interface
```
Inside the package create two new folders to seperate the `.msg` and `.srv`
files.

## Creating Custom Definitions

In the msg directory that you just created create a file named `Num.msg`.
Inside the file you will define its data structure with:
```
int64 num
```

This custom message will transfer a single 64 bit int.

In the `srv` directory create a file called `AddThreeInts.srv` with following
structureL:
```
int64 a
int64 b
int64 c
----------
int sum
```
This file defines the structure of the request and the response. This service
requires three int names `a`, `b` and `c` and it will return an in called `sum`

To convert the `.srv` and `.msg` files to C++ code add the following line to the
`CMakeLists.txt`:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
 )

```

Interface rely on `rosidl_default_generators` to generate language specific
code, must declare it as a dependecy in `package.xml`:
```
<build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>

```

Build the `tutorial_interface` package from the root, `dev_ws`, of the workspace using `colcon`. Now you will be able to use the interfaces in the ROS package.
To insure the interface is working correctly use the `ros2 interface show command` in the
`dev_ws/src` directory.
```
ros2 interface show tutorial_interfaces/msg/Num
```
The terminal response should be:
```
int64 num
```
Use the same command to show the contents of the `.srv` file. It should return:
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


After making a copy of the files execute the following commands in the Pub/Sub package to overwrite `publisher_member_function.cpp` and `subscriber_member_function.cpp`:
```
wget -O publisher_member_function.cpp (URL to file)
wget -O subscriber_member_function.cpp (URL to file)
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

In `package.xml` add the following line:
```
<depend>tutorial_interfaces</depend>
```
After editing the files navigate back to `dev_ws` and rebuild `cpp_pubsub` using
`colcon`. Then open two terminals and run the `talker` and `listener`. Since `Num.msg`
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
and add the following line to `package.xml`:
```
<depend>tutorial_interfaces</depend>
```
After making the changes navigate to `dev_ws` and build the `cpp_svicli` package
with `colcon`.

Once the build is complete open two terminals and run the following to test the
service and client:
```
ros2 run cpp_srvcli server

ros2 run cpp_srvcli client 2 3 1
```
