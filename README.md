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

## Solving the persistent robot deadlocks

To enhance multi-robot exploration stability and prevent "deadlock" scenarios (where robots block each other in narrow corridors therefor the mapping stops), we implemented the following optimizations:

### Navigational Parameter Tuning
We tuned the `navigation_1.yaml` and `navigation_2.yaml` files to improve robot mobility:
* **`footprint_clearing_enabled`**: Set to `True` to allow clearing of stale Lidar data.
* **`inflation_radius`**: Reduced from `0.4m` to `0.25m` to create thinner safety zones.
* **`cost_scaling_factor`**: Increased to `15.0` to encourage navigation closer to obstacles.
* **`global_costmap` robot_radius**: Unified to `0.2m` to match the local costmap, preventing planning failures in tight corners.

### Intelligent Deadlock Resolution (`explore_map_any_robots.py`)
We integrated a robust state machine into the exploration logic to handle robot collisions and unreachable targets:

1.  **Stuck Detection**: The node monitors robot progress. If a robot stays within 5 meters of a target for over 10 seconds, it is marked as "stuck".
2.  **Costmap Clearing**: Upon detecting a stuck robot, the node automatically invokes the `nav2_msgs/srv/ClearEntireCostmap` service to reset both local and global costmaps.
3.  **Retreat Maneuver**: The robot is commanded to navigate back to its `home_pose` for 15 seconds to vacate the corridor, allowing the other robot to pass.
4.  **Temporal Blacklisting**: Targets that cannot be reached are added to a timestamped dictionary. They are ignored for 60 seconds, allowing the environment to change (i.e., the other robot to move away) before re-attempting.

### Implementation Details
The following dictionaries and functions were added to the `MultiRobotExplorer` class in `explore_map_any_robots.py`:

* **Data Structures**: 
    ```python
    self.blacklists = {}          # Dictionary mapping (y, x) to timestamp
    self.retreat_until = {}       # Dictionary mapping robot_name to escape expiration time
    ```
* **Key Functions**:
    * `clean_expired_blacklists()`: Periodically removes entries from `self.blacklists` older than 60s.
    * `check_and_blacklist_stuck_targets()`: The core logic that triggers the costmap clearing service, manages the retreat-to-home command, and updates the blacklist.