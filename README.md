# cs4013_oliver

## Overview

TODO

## How to Run 🏎️

We assume that the user has completed the following initialization procedure before running this code:

* Install ROS Melodic on Ubuntu 18.04
* Create and source a catkin workspace
* Install Turtlebot packages and dependencies
* Build the catkin workspace

SSH -Y into 5 terminals, and run these 5 commands in the given order in each terminal:

1. `roslaunch turtlebot_bringup minimal.launch`
2. `roslaunch turtlebot_bringup 3dsensor.launch`
3. `roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/p1_world/scripts/real_world.yaml`
4. `roslaunch turtlebot_rviz_launchers view_navigation.launch`
5. `rosrun tf tf_echo /map /base_link`

Finally, on the desktop terminal, run:

`python tour_robot.py`

Note: The given repository does not contain the entire catkin workspace. Assuming the same default file hierarchy of a catkin workspace, the files in our home directory correspond to the `src` directory, though only relevant sub-directories are included in this repository.

The section below will elaborate on and discuss key files we have modified to make our implementation possible.

## Notable Code

### 1. Launch 📁
* **[minimal.launch](https://github.com/nxm23763/cs4013_oliver/blob/main/turtlebot_bringup/launch/minimal.launch):** Sets up the Turtlebot parameters, includes configurations for the robot and mobile base, simulates a fake laser using nodelets, configures Gmapping for SLaM (Simultaneous Localization and Mapping), and launches RViz for visualization.
* **[3dsensor.launch](https://github.com/nxm23763/cs4013_oliver/blob/main/turtlebot_bringup/launch/3dsensor.launch):** Configures a 3D sensor nodelet for Turtlebot, which allows customization of processing modules (RGB, IR, depth, etc.), depth registration, and laser scan processing.
* **[amcl_demo.launch](https://github.com/nxm23763/cs4013_oliver/blob/main/turtlebot_gazebo/launch/amcl_demo.launch):** Sets up a map server for Turtlebot, specifies parameters for localization using Adaptive Monte Carlo Localization (AMCL), and configures the Move Base system.
* **[view_navigation.launch](https://github.com/nxm23763/cs4013_oliver/blob/main/turtlebot_rviz_launchers/launch/view_navigation.launch):** Configures visualization of the Turtlebot in RViz during map building or navigation using the ROS navigation stack.

### 2. Scripts 📁

### 3. Worlds 📁 //is this needed?
