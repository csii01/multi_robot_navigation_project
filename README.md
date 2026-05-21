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
