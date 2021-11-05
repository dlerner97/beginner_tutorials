# beginner_tutorials
ROS tutorial for ENPM808X class. This tutorial contains two nodes. One node publishes a certain string to a "chatter" topic. The other node subscribes to the "chatter" topic and prints the results. Furthermore, node 1 is also a server and accepts requests to change the string that it publishes. Node 2 is a service client and changes the string once after 5 seconds just so I can learn how to do this in code. Lastly, one can also make a service request from the terminal. Please see the **Running Beginner Tutorial** section below. 

## Submission Items
For the rqt console screenshot and static code checks, see the Code Results folder inside the repo.

# Run Instructions

## Dependencies
1. **C++11** or greater
2. **ROS Noetic** or **Melodic**. This package was generated on a system with ROS Noetic. However, the code, CMakeLists, package.xml, as well as the running instructions will work on ROS Melodic as well.
3. **catkin build** system. The running instructions use the "catkin build" system rather than the "catkin_make." These two ROS package managers are very similar but catkin build is newer and supposedly better. Since this repo only contains the individual package (i.e. src code), one can still use catkin_make as their manager. Except for replacing `catkin build` to `catkin_make` line in the **Running the Beginner Tutorial**, no other instructions should be affected. See online tutorials for more information.

### Install ROS

Follow the [ROS installation guide](http://wiki.ros.org/ROS/Installation) to install ros on your system.

### Create a catkin workspace

This section only applies if no catkin workspaces have already been installed on your system. If one is available, please skip this section and head to **Running the Beginner Tutorial** section

In a terminal, create a workspace name of your choosing. Generally, the standard is to add a "_ws" to the end of a desired name. For this example we'll use the name `catkin_ws`.

```bash
cd ~
mkdir catkin_ws
cd catkin_ws
mkdir src
```

Type the following command into your terminal to source your new catkin workspace into the bashrc

```bash
echo 'source <path to workspace>/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### Running the Beginner Tutorial

Open a terminal and clone the repo into a `src` folder within the desired catkin workspace. Use
```bash
git clone git@github.com:dlerner97/beginner_tutorials.git
```
for an ssh clone and 
```bash
git clone https://github.com/dlerner97/beginner_tutorials.git
```
for an https clone. Then,
```bash
# Go into the catkin_ws folder
cd ..

catkin build
```

Open 2 additional terminals. In one,
```bash
source <path to workspace>/<workspace name>/devel/setup.bash
roslaunch beginner_tutorials TalkerListener.launch default_string:='"desired string"'
```

Note a few things. First, the desired string has a default argument and still will work without this param. Second, **make sure you use the quotations as shown above**. The use of spaces in a parameter is strange to the CMD and one must input their desired string exactly as it is shown above. Lastly, this launch file spins up two nodes: a publisher and a subscriber. In my implementation, the subscriber also makes a service call after 5 seconds (changing the string output of the publisher). To make a custom service call bring up the other terminal and follow the instructions below.

```bash
source <path to workspace>/<workspace name>/devel/setup.bash
rosservice call /set_output_string '"new desired string"'
```