# Oliver: A Speech-Oriented Tour Guide Robot

*University of Oklahoma, CS 4013 - Intro to Intelligent Robotics*

*Authors: Justus Sommer, Minh Tran, Jacob Pierce, Creighton Cornelison, Natalia Mora*

## Overview

This repository contains the source code for an implementation of an NLP-augmented tour guide robot on a Kobuki Turtlebot.

Our robot, named "Oliver," is able to listen for and respond to user commands. Oliver can complete the following commands:

* **`AUTOMODE`**: Oliver autonomously wanders around the room, avoiding obstacles. No target node is assigned.
* **`TO_<NODE>`**: Oliver travels to specified target node.
* **`WHERE`**: Oliver declares its current location.
* **`HERE_<NODE>`**: Oliver listens to user's specified node and sets it as its current location.
* **`HALT`**: Oliver will stop moving.

Please see our project report (to be linked here soon) for more information on our development lifecycle, process, and rationale.

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

`roslaunch p1_world turtle_main.launch`

Note: The given repository does not contain the entire catkin workspace. Assuming the same default file hierarchy of a catkin workspace, the files in our home directory correspond to the `src` directory, though only relevant sub-directories are included in this repository.

The section below will elaborate on and discuss key files we have modified to make our implementation possible.

## Notable Code

### 1. Launch 📁
* **[minimal.launch](https://github.com/nxm23763/cs4013_oliver/blob/main/turtlebot_bringup/launch/minimal.launch):** Sets up the Turtlebot parameters, includes configurations for the robot and mobile base, simulates a fake laser using nodelets, configures Gmapping for SLaM (Simultaneous Localization and Mapping), and launches RViz for visualization.
* **[3dsensor.launch](https://github.com/nxm23763/cs4013_oliver/blob/main/turtlebot_bringup/launch/3dsensor.launch):** Configures a 3D sensor nodelet for Turtlebot, which allows customization of processing modules (RGB, IR, depth, etc.), depth registration, and laser scan processing.
* **[amcl_demo.launch](https://github.com/nxm23763/cs4013_oliver/blob/main/turtlebot_gazebo/launch/amcl_demo.launch):** Sets up a map server for Turtlebot, specifies parameters for localization using Adaptive Monte Carlo Localization (AMCL), and configures the Move Base system.
* **[view_navigation.launch](https://github.com/nxm23763/cs4013_oliver/blob/main/turtlebot_rviz_launchers/launch/view_navigation.launch):** Configures visualization of the Turtlebot in RViz during map building or navigation using the ROS navigation stack.

### 2. Scripts 📁
* **[turtle_main.py](https://github.com/nxm23763/cs4013_oliver/blob/main/p1_world/scripts/turtle_main.py):** The main file for our Oliver ROS robot program, containing all functionality for Oliver's human interaction, navigation, path planning, and localization tasks. Functionalities of the implemented features include:
  * **Navigation**: The `GoToPose` class handles Oliver's navigation to specific poses, and uses ROS navigation stack to steer the robot towards given points, or nodes, to travel towards.
  * **Path Planning**: The utility functions at the top of our script, `total_distance`, `manhattan_distance` and `tsp_manhattan`, implement the Traveling Salesman problem to calculate the optimal path between any set of nodes to minimize time, resource and energy consumption. These functions support the navigation step above.
  * **Command Definitions**: The commands seen in the overview above are defined within this file, which allows Oliver to interpret incoming voice requests as actionable tasks to complete. We implemented a command library, which allows the user more free range on how to personalize their interaction with Oliver without sacrificing the authenticity of natural language, forwarding our team's mission for an NLP-driven solution that ameliorates the HRI experience.
  * **Voice Processing and Recognition**: The `AudioRecorder` class handles audio processing from the microphone using ROS, allowing Oliver to listen to and recognize spoken word. It also handles voice recognition by using the Google Automatic Speech Recognition API, which allows Oliver to hear from its user and respond accordingly by mapping input to the command library above.
  * **User Interaction**: Oliver is equipped to talk to the user through a sound client to improve the HRI experience. Oliver introduces itself and provides audible feedback on its inputs. Oliver will talk about a specific tour landmark (node) when it takes a user on the tour, and it both gives and receives feedback on its localization data upon request. Additionally, Oliver provides feedback if the node was not reached in order to round out the optimal HRI experience.
* **[ai.py](https://github.com/nxm23763/cs4013_oliver/blob/main/p1_world/scripts/ai.py):** Runs Python 3.6.9 unlike the rest of our source code for OpenAI compatibility. Uses the OpenAI API to ask a question to the GPT-3.5 Turbo model. Prints the generated response after processing the user's question and handles exceptions such as rate limits or invalid requests. The question is passed as a command-line argument and is activated by our main file, `turtle_main.py`.
