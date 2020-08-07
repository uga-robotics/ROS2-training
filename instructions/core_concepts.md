# Nodes, Topics, Services, Actions, and Parameters in ROS2
In this tutorial we'll go into detail on the core features of a ROS2 system and how they interact: **nodes**, **topics**, **services**, **actions**, and **parameters**.

## Nodes
As described in our overview, nodes are fundamental ROS 2 element that serves a single, modular purpose in a robotics system. Each node in ROS should be responsible for a single purpose (e.g. one node for controlling wheel motors, one node for controlling a laser range-finder, etc). Each node can send and receive data to other nodes via topics, services, actions, or parameters. A full robotic system is comprised of many nodes working in concert.

### Starting and Finding Nodes
In ROS2, a single executable (C++ program, Python program, etc.) can contain one or more nodes. These executables can be run using the command:
```
ros2 run <package_name> <executable_name>
```
where ```package_name``` is the name of the ROS2 package and ```executable_name``` is the name of the executable (e.g. ```turtlesim turtlesim_node```).

If turtlesim is not already running, go ahead and start it by using this command:
```
ros2 run turtlesim turtlesim_node
```

As we have done before, you can find nodes in a ROS2 system by using the command:
```
ros2 node list
```

If you open a new terminal (making sure to have the proper setup file sourced) and run ```ros2 node list``` you should see the name for our turtlesim node:
```
/turtlesim
```

### Remapping
In ROS2, remapping allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values. In the last tutorial, you used remapping on ```turtle_teleop_key``` to change the default turtle being controlled.

To demostrate, we'll create a new ```turtlesim``` node remapped to have the new name ```/my_turtle```:
```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```
If you run ```ros2 node list``` again, you'll see the new name:
```
/turtlesim
/my_turtle
```

### Getting More Information About a Node
Now that we can access the names of nodes in our system, we can get all sort of other useful information about the using the ```ros2 node info <node_name>``` command where ```node_name``` is the name of the node you want to look into. To examine our newly created ```/my_turtle``` node, execute the command:
```
ros2 node info /my_turtle
```
This command will return a list of subscribers, publishers, services, and actions (the ROS graph connections) that interact with that node. The output should look like this:
```
/my_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```
You'll learn more about these connections and what they mean in the next sections.

## Topics
ROS 2 breaks complex systems down into many modular nodes. **Topics** are a vital element of the ROS graph that act as a bus for nodes to exchange messages. A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics. Topics are one of the important ways that data moves between nodes, and therefore between different parts of the system.

By now, you should be pretty familiar with how to start up turtlesim. Simply open up a couple of sourced terminals and enter the commands ```ros2 run turtlesim turtlesim_node``` and ```ros2 run turtlesim turtle_teleop_key``` respectively.

### Introspection Tools
In this section we'll get into using some introspection tools, as they will be helpful when learning about topics. The first introspection tool we'll use is one you're already aquainted with: rqt_graph. rqt_graph does visually what all of the command-line introspection tools do, but visually, so it will be a great backdrop for learning about them.

#### rqt_graph
First, open up rqt_graph in a new sourced terminal, set the ```group``` variable to 0, set the visibility to ```Nodes/Topics (active)``` from the drop down menu and hit the refresh button next to it. You should now see the following graph:
![rqt graph 2](resources/rqtgraph2.png)

In the image above, the ovals or circles are nodes, and the rectangles are topics. If you hover your mouse over a node or topic, you'll see the highlighting like in the above picture. The blue indicates a ```publisher``` or sender on a particular topic, red indicates a topic/topic name, and green indicates a ```subscriber``` or reciever on the topic. This highlighting is important for dissecting large or complex systems. The graph is depicting how the /turtlesim node and the /teleop_turtle node are communicating with each other over a topic. The /teleop_turtle node is publishing data (the keystrokes you enter to move the turtle around) to the /turtle1/cmd_vel topic, and the /turtlesim node is subscribed to that topic to receive the data.

If you uncheck all of the boxes under the **Hide** section in rqt_graph, the topics in the graph should match those found when running ```ros2 topic list```, a command-line introspection tool (aside from the topics generated for the actions interface). For now though, leave these boxes checked to avoid confusion.

#### Echo
While we're already familiar with ```ros2 topic list```, we haven't encountered ```ros2 topic echo <topic_name>```, which allows the developer to see the data being published on a topic ```topic_name``` in real time! If we do this on the ```/turtle1/cmd_vel``` topic, we won't see anything at first, because nothing has been published on the topic while we've been looking at it. As soon as we make the turtle move, we should see some messages in the format:
```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
  ---
```

#### Info
As we've seen before with the ```ros2 node info <node_name>``` command, the ```ros2 topic info <topic_name>``` command will show us some general information about a given topic. for example if you run ```ros2 topic info /turtle1/cmd_vel``` you should see the output:
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscriber count: 2
```
This illustrates that topics are not one-to-one only. There can be as many (reasonable) number of subscribers and publishers on a topic at any given time.

#### Interface Show
Nodes send data over topics using messages. Publishers and subscribers must send and receive the same type of message to communicate. by running the command ```ros2 topic list -t``` we can see that the topic ```/turtle1/cmd_vel``` is of the type ```geometry_msgs/msg/Twist```. We can get some more information on what these messages look like by using the ```ros2 interface show geometry_msgs/msg/Twist``` command which gives us the result:
```
# This expresses velocity in free space broken into its linear and angular parts.

    Vector3  linear
    Vector3  angular
```
If we then execute the command again, only this time as ```ros2 interface show geometry_msgs/msg/Vector3``` we'll get this output:
```
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
```
If you recall the data we saw when echoing the ```/turtle1/cmd_vel``` topic, you'll find this structure very familiar!

#### Pub
Now that you have the message structure, you can publish data onto a topic directly from the command line using ```ros2 topic pub <topic_name> <msg_type> '<args>'``` with the topic name and type as arguments, and the ```'<args>'``` section the message you want to publish in [YAML](https://en.wikipedia.org/wiki/YAML) syntax, a format we'll be using a lot in ROS2. ```--once``` is an optional argument used to publish the message only once, then exit:
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
After entering the previous command, the turtle should have moved like so:
![turtlesim semicircle](resources/turtlesim2.png)

If we drop the ```--once``` tag and add ```--rate 1```, the command will publish our message at a rate of 1 HZ (once per second), causing the turtle to move in a circle:
![turtlesim circle](resources/turtlesim3.png)

#### Hz
The last introspection tool we'll be looking at is ```ros2 topic hz <topic_name>```, which reports the rate at which data on the topic is being published. If we use this command on the ```/turtle1/pose``` topic we'll see:
```
average rate: 62.509
    min: 0.015s max: 0.016s std dev: 0.00038s window: 64
```
which gives us some statistics on the rate data is being published on that topic. This is especially useful when dealing with networked or [multi-agent](https://en.wikipedia.org/wiki/Multi-agent_system) systems, as network interference or slowdown can cause wierd and unforseen issues on a ROS2 system, and the ```echo``` tool could help diagnose this scenario.

## Services
