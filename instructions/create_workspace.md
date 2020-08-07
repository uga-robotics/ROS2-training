# An overview of ROS2 Workspaces
In this tutorial, we'll create our own workspace. A workspace is a directory that contains ROS2 packages. Workspaces can be layered to achieve desired functionality.

## Prerequisites
Before we begin, source your ROS2 setup file. Check if Colcon is installed by running:
```
colcon version-check
```
your output should look like this:

 ```
 colcon-argcomplete 0.3.3: up-to-date
 colcon-bash 0.4.2: up-to-date
 colcon-cd 0.1.1: up-to-date
 colcon-cmake 0.2.23: newer version available (0.2.24)
 colcon-core 0.5.10: newer version available (0.6.0)
 colcon-defaults 0.2.5: up-to-date
 colcon-devtools 0.2.2: up-to-date
 colcon-library-path 0.2.1: up-to-date
 colcon-metadata 0.2.4: up-to-date
 colcon-notification 0.2.13: up-to-date
 colcon-output 0.2.11: up-to-date
 colcon-package-information 0.3.3: up-to-date
 colcon-package-selection 0.2.7: newer version available (0.2.8)
 colcon-parallel-executor 0.2.4: up-to-date
 colcon-pkg-config 0.1.0: up-to-date
 colcon-powershell 0.3.6: up-to-date
 colcon-python-setup-py 0.2.5: newer version available (0.2.6)
 colcon-recursive-crawl 0.2.1: up-to-date
 colcon-ros 0.3.18: newer version available (0.3.19)
 colcon-test-result 0.3.8: up-to-date
 colcon-zsh 0.4.0: up-to-date
 ```
 If you receive an error run the following command to install colcon:
 ```
 sudo apt install python3-colcon-common-extensions
 ```

 Once Colcon is installed ensure that git is installed by running:
 ```
 git --version
 ```
 If you receive an error follow [this tutorial](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) to install git.

 After ensuring git is installed, run the ``` rosdep``` command if you revieve an error follow [this tutorial](https://wiki.ros.org/rosdep#Installing_rosdep). Once all of that is completed and you have completed the [turtlesim/rqt tutorial](turtlesim_rqt.md) you are ready to create your first workspace.

 ## Creating a Workspace

 Every workspace should have its own directory with the same name as the workspace. For this tutorial we will be creating the **dev_ws** workspace. To create this directory run:
 ```
 mkdir -p ~/dev_ws
 ```
 after running the command list the files in the current directory using the ```ls``` command the output of which should include the folder you just created. Navigate into the new directory using the ```cd``` command:
 ```
 cd dev_ws
 ```
 and create another directory called **src** and navigate into it. The packages created in a workspace should be placed in a **src** directory.

 While inside the **src** directory clone the git repository containing exercise files:
 ```
 git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
 ```
 List the files in the directory using the ```ls``` command, you should see:
 ```
 ros_tutorials
 ```
 Navigate into the **ros_tutorials** folder using ```cd``` and list the contents of the directory using ```ls``` You should see.

 ```
 roscpp_tutorials  rospy_tutorials  ros_tutorials  turtlesim
 ```
 We will uses these files to practice putting a workspace together.

 ## Resolving dependencies
 With files in your **src** folder we can start creating a fully functional workspace. We begin my resolving the dependencies in our packages. Dependency resolution ensures we have all the files we need to build our workspace. Using ```rosdep``` command we can automatically install all the dependencies that our packages need. Navigate the parent directory of the present working directory using ```cd ..``` and run the following command:
 ```
 rosdep install --from-paths src --rosdistro foxy -y
 ```
 The ```rosdep``` command will crawl the **src** directory looking for dependencies declared in **package.xml**, which you will learn about later, and will install them. Once the command has finished executing you will be able to build your workspace with colcon.

 ## Building your Workspace

 To build your workspace run the following command in the **dev_ws** directory:
 ```
 colcon build
 ```
 Once the build is finished list the contents of the directory. You will see:
 ```
 build  install  log  src
 ```
 The **install** directory will contain your workspace's setup files.

 ## Sources your Workspace

 Open a new terminal to separate your current workspace from the one that just created, this is important for all workspaces you create to prevent complex issues, and source the ROS2 workspace again. Navigate to the **dev_ws** directory and source your overlay:
 ```
 . install/local_setup.bash
 ```
 *local_setup.bash* only adds the packages available in the current overlay to the environment, to source the current overlay and its underlays use *setup.bash*.

 You will now be able to run **turtlesim**.

 ## Modifying the overlay

 To test if you are using your newly created workspace, lets change the title of the Turtlesim window. Nagigate to:
 ```
 dev_ws/src/ros_tutorials/turtlesim/src
 ```
 and open the *tutle_frame.cpp* and edit line 52 to say:
 ```
 setWindowTitle("My workspace");
 ```
 and rebuild the workspace using the first terminal. Return to the second terminal and rerun **turtlesim** it should "My workspace" for the title of the window. Run **tutlesim** in the first terminal, the title of the window should be "TurtleSim".
