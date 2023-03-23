# `wayp_plan_tools` `ROS 2` package
Waypoint and planner tools for `ROS 2`

Planner / control nodes:
- `single_goal_pursuit`: Pure pursuit (for vehicles)
- `multiple_goal_pursuit`: Multiple goal pursuit for vehicles an implementation of our [paper](https://hjic.mk.uni-pannon.hu/index.php/hjic/article/view/914)

Waypoint nodes:
- `waypoint_saver`: saves the waypoint to a csv
- `waypoint_loader`: loads the waypoint from a csv to a ROS 2 topic


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