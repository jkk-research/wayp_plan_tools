# `wayp_plan_tools` `ROS 2` package
Waypoint and planner tools for `ROS 2` with mininal dependencies.

Planner / control nodes:
- `single_goal_pursuit`: Pure pursuit (for vehicles / robots)
- `multiple_goal_pursuit`: Multiple goal pursuit for vehicles / robots an implementation of our [paper](https://hjic.mk.uni-pannon.hu/index.php/hjic/article/view/914)

Waypoint nodes:
- `waypoint_saver`: saves the waypoints to a csv
- `waypoint_loader`: loads the waypoints from a csv to a ROS 2 topic


## Build

It is assumed that the workspace is `~/ros2_ws/`.

### `Terminal 1` 🔴 clone

```
cd ~/ros2_ws/src
git clone https://github.com/jkk-research/wayp_plan_tools
```

### `Terminal 1` 🔴 build
```
cd ~/ros2_ws
colcon build --packages-select wayp_plan_tools
```

### `Terminal 2` 🔵 run
```
source ~/ros2_ws/install/local_setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch wayp_plan_tools waypoint_saver.launch.py
```
# `single_goal_pursuit` node
The "classic" pure pursuit implementation

# `multiple_goal_pursuit` node 
Multiple goal pursuit for vehicles / robots, an implementation of our [paper](https://hjic.mk.uni-pannon.hu/index.php/hjic/article/view/914)

# `waypoint_to_target` node
Reads the waypoint array and speeds, from that it creates singel or mutliple goal points.

It also provides a `/metrics_wayp` array topic with the following  elements:

| Array element | Meaning 
| :--- | :--- 
|`[0]` | current lateral distance to the waypoint
|`[1]` | average lateral distance over time
|`[2]` | current waypoint ID
|`[3]` | target waypoint ID
|`[4]` | target waypoint longitudinal distance 
# `waypoint_saver` node 
Saves the waypoints to a csv.
# `waypoint_loader` node
Loads the waypoints from a csv to a ROS 2 topic.




# Cite & paper

If you use any of this code please consider citing the [paper](https://hjic.mk.uni-pannon.hu/index.php/hjic/article/view/914):

```bibtex
@Article{horvath2020multigoalpursuit, 
    title={Theoretical background and application of multiple goal pursuit trajectory follower}, 
    volume={48}, 
    url={https://hjic.mk.uni-pannon.hu/index.php/hjic/article/view/914}, 
    DOI={10.33927/hjic-2020-03}, 
    number={1}, 
    journal={Hungarian Journal of Industry and Chemistry}, 
    author={Horváth, Ernő and Pozna, Claudiu and Kőrös, Péter and Hajdu, Csaba and Ballagi, Áron}, 
    year={2020}, 
    month={Jul.}, 
    pages={11–17} 
}
```

![](csv/rviz2waypoint01.png)