# Kooperatív 2 robotos  Térképezési Projekt kiterjesztése több világra
## Multi-robot exploration
**Important:** first launch the launch file *(multirobot_navigation_slam_toolbox.launch.py)*, and then run the exploration logic **in a separate terminal** *(explore_map2)*

**First:**

```bash
ros2 launch multi_robot_navigation multirobot_navigation_slam_toolbox.launch.py
```

**Then in a separate terminal window:**
```bash
ros2 run multi_robot_explore explore_map2
```

## Python dependency
I'm not sure, but if stuff isn't working out for you, you could try installing the Python package shapely.

First make sure that your system is up-to-date:

```bash
sudo apt update
```

If it is needed, you can upgrade:

```bash
sudo apt upgrade
```

Then install dependency:

```bash
sudo apt install python3-shapely
```

Don't forget to build (```colcon build```) and source the bash file (```source install/setup.bash```)!

## Improved explore: any robots
The exploration logic was modified: instead of two hardcoded robots, this new exploration logic now scans for robots automatically.
I created a new file for this exploration logic: ***multi_robot_explore/multi_robot_explore/explore_map_any_robots.py***, which can be started via this command:
```bash
ros2 run multi_robot_explore explore_map_any_robots
```
**As it handles available robots automatically, it is not important if you start this node before or after the launchfile.**

### Scanning procedure:
The scanning procedure is started every 10 seconds via a timer callback. From all available topics it finds the ones ending with *"/robot_description"*, assuming every robot publihes this kind of topic. If the robot name is not contained in the robot names dictionary, the robot is added to the appropriate internal dictionaries, to handle this new robot.

### Marker colors
It is important to have distinct marker colors for each robot. Instead of the hardcoded colors for *robot_1* and *robot_2*, a distinct color is generated from the robot name. A uniform hash function is used to map the robot names to hue values between 0 and 1. This means, that every robot will have a different marker color, and the same robot will always get the same marker color, at every launch.

### Blacklisting
Originally a potentially unreachable goal position was only blacklisted, if the robot was closer to it, then 1 m. However, this made robots stuck, as there can be unreachable positions further away, so this threshold was increased to 5 m, which resolved the robot-is-stuck issue.
