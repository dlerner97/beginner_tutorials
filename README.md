# beginner_tutorials
ROS tutorial for ENPM808X class.

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
source <path to workspace>/<workspace name>/devel/setup.bash

# Start a ros core
roscore
```

Open 2 additional terminals. In one,
```bash
source <path to workspace>/<workspace name>/devel/setup.bash
rosrun beginner_tutorials talker
```

In another,
```bash
source <path to workspace>/<workspace name>/devel/setup.bash
rosrun beginner_tutorials listener
```