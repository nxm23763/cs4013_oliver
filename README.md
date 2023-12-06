# cs4013_oliver

## How to Run

SSH -Y into 5 terminals, and run these 5 commands in the given order in each terminal:

1. `roslaunch turtlebot_bringup minimal.launch`
2. `roslaunch turtlebot_bringup 3dsensor.launch`
3. `roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/p1_world/scripts/real_world.yaml`
4. `roslaunch turtlebot_rviz_launchers view_navigation.launch`
5. `rosrun tf tf_echo /map /base_link`

Finally, on the desktop terminal, run:

`python tour_robot.py`

## Files

### 1. Launch
* **minimal.launch:**

### 2. Scripts

### 3. Worlds
