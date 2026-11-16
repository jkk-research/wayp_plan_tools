# `wayp_plan_tools` `ROS 2` package
Waypoint and planner tools for `ROS 2` with minimal dependencies. [![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

Tutorial (simulation + controllers): [jkk-research.github.io/latest/workshops/control_sim](https://jkk-research.github.io/latest/workshops/control_sim/)

Planner / control nodes:
- `single_goal_pursuit`: Pure pursuit (for vehicles / robots), a simple cross-track error method
- `multiple_goal_pursuit`: Multiple goal pursuit for vehicles / robots an implementation of our [paper](https://hjic.mk.uni-pannon.hu/index.php/hjic/article/view/914)
- `stanley_control`: Stanley controller, a heading error + cross-track error method
- `follow_the_carrot`: Follow-the-carrot, the simplest controller

Waypoint nodes:
- `waypoint_saver`: saves the waypoints to a csv
- `waypoint_loader`: loads the waypoints from a csv to a ROS 2 topic
- `waypoint_to_target`: translates the global waypoint array to local target waypoint(s), thus it makes possible to use the controller nodes in a standalone way


## Build

It is assumed that the workspace is `~/ros2_ws/`.

### `Terminal 1` 🔴 clone

```
cd ~/ros2_ws/src
```
```
git clone https://github.com/jkk-research/wayp_plan_tools
```

### `Terminal 1` 🔴 build
```
cd ~/ros2_ws
```
```
colcon build --packages-select wayp_plan_tools --symlink-install
```

### `Terminal 2` 🔵 run
```
source ~/ros2_ws/install/setup.bash
```
```
ros2 launch wayp_plan_tools waypoint_saver.launch.py
```

# Control nodes
In this project the `single_goal_pursuit`, the `multiple_goal_pursuit`, `stanley_control` and the `follow_the_carrot` controllers are implemented. A high level overview about the algorithms is visible on the following figure:

![](csv/control_overview01.svg)

# `single_goal_pursuit` node
The "classic" pure pursuit implementation
![](csv/control_pure_pursuit01.svg)

# `multiple_goal_pursuit` node 
Multiple goal pursuit for vehicles / robots, an implementation of our [paper](https://hjic.mk.uni-pannon.hu/index.php/hjic/article/view/914)

It extends pure pursuit from one goal point to several. The node reads the goal points from `waypoint_topic` (`PoseArray` in the vehicle frame, `base_link` = center of the rear axle). For each goal point $i$ it computes the pure pursuit steering angle $\delta_i = \arctan\frac{2 L \sin\alpha_i}{l_{d,i}}$. Here $L$ is the `wheelbase`, $\alpha_i$ is the angle of the goal point and $l_{d,i}$ is its distance. The steering command is the average of these angles, $\delta = \frac{1}{n}\sum_{i=1}^{n}\delta_i$. Several goal points at different distances smooth the steering: the near points correct the lateral error and the far points anticipate the curves. With one goal point the node is the same as `single_goal_pursuit`. The speed comes from `pursuitspeedtarget`. The output is published on `cmd_vel` (`linear.x` = speed, `angular.z` = steering angle [rad]).

# `stanley_control` node
Stanley lateral controller, implemented according to the original paper: S. Thrun et al., *Stanley: The Robot that Won the DARPA Grand Challenge*, Journal of Field Robotics 23(9), 2006, Section 9.2 ([pdf](https://robots.stanford.edu/papers/thrun.stanley05.pdf)).

The steering law (Eq. 7 in the paper):

$$\delta(t) = \psi(t) + \arctan\frac{k\,x(t)}{u(t)}$$

- $\psi(t)$: heading error, the orientation of the nearest path segment relative to the vehicle's orientation
- $x(t)$: cross-track error, the lateral distance of the **center of the front axle** from the nearest point of the path (signed, positive if the path is on the left)
- $u(t)$: vehicle speed
- $k$: gain, for small errors $x(t) \approx x(0)\,e^{-kt}$ (Eq. 9), so $k$ sets the convergence rate

With zero cross-track error the front wheels are steered parallel to the path; with a large error the $\arctan$ term turns the wheels toward the path (at most $\pm 90°$).

``` r
ros2 launch wayp_plan_tools stanley_control.launch.py
```

# `follow_the_carrot` node
Follow-the-carrot, the simplest controller
![](csv/control_follow_the_carrot01.svg)

# `waypoint_to_target` node
Reads the waypoint array and speeds, from that it creates single or multiple goal points.

It also provides a `/metrics_wayp` array topic with the following  elements:

| Array element | Meaning | Const
| :--- | :--- | :---
|`[0]` | current lateral distance to the waypoint (signed, cross-track error) | `CUR_LAT_DIST_SIGNED`
|`[1]` | current lateral distance to the waypoint (absolute value) | `CUR_LAT_DIST_ABS`
|`[2]` | average lateral distance over time | `AVG_LAT_DISTANCE`
|`[3]` | maximum lateral distance over time | `MAX_LAT_DISTANCE`
|`[4]` | current waypoint ID | `CUR_WAYPOINT_ID`
|`[5]` | target waypoint ID | `TRG_WAYPOINT_ID`
|`[6]` | target waypoint longitudinal distance (similar to lookahed distance, but at a waypoint) | `TRG_WAY_LON_DIST`
|`[7]` | actual lookahead distance | `ACT_LOOK_DIST`
|`[8]` | heading difference between the closest waypoint and the vehicle [rad] (despite its name, not a distance) | `CUR_CROSS_TRACK`


<img src="csv/lookahead01.svg" width=60% />

# `waypoint_saver` node 
Saves the waypoints to a csv. Important parameters are `file_name` and `file_dir`. Set from terminal or from a [launch file](launch/waypoint_saver.launch.py)

``` py
ros2 run wayp_plan_tools waypoint_saver --ros-args -p file_name:=tmp1.csv -p file_dir:=/mnt/bag/waypoints
```

# `waypoint_loader` node
Loads the waypoints from a csv to a ROS 2 topic. Set from terminal or from a [launch file](launch/waypoint_loader.launch.py)

``` py
ros2 run wayp_plan_tools waypoint_loader --ros-args -p file_name:=tmp1.csv -p file_dir:=/mnt/bag/waypoints
```


# Usage with a simulator

Please refer to [github.com/jkk-research/sim_wayp_plan_tools](https://github.com/jkk-research/sim_wayp_plan_tools).

A step-by-step tutorial is available at [jkk-research.github.io/latest/workshops/control_sim](https://jkk-research.github.io/latest/workshops/control_sim/). It covers the Gazebo Fortress + ROS 2 Humble simulation, loading waypoints, `waypoint_to_target`, running the controllers (`single_goal_pursuit`, `multiple_goal_pursuit`, `stanley_control`, `follow_the_carrot`), tuning with `rqt_reconfigure` and visualization in RViz2.

![](https://raw.githubusercontent.com/jkk-research/sim_wayp_plan_tools/main/img/gz_rviz01.gif)

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
