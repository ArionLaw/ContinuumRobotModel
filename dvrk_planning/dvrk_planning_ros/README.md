# dvrk_planning_ros

## Dependencies
Insall ROS: http://wiki.ros.org/noetic/Installation/Ubuntu
Install catkin tools https://catkin-tools.readthedocs.io/en/latest/installing.html

## Build
Make workspace so folder structure is <your_workspace>/src/dvrk_planning

source ros:

```
source /opt/ros/noetic/setup.bash

or

source /opt/ros/<your_distro>/setup.bash
```

build:

```
cd <your_workspace>
catkin build
source devel/setup.bash
```

## Launch dvrk_planning_node

Launch the node:

```bash
    roslaunch dvrk_planning_ros dvrk_planning.launch
```

Call the ik service:

```bash
    rosservice call /psm/compute_ik "{tf_stamped:{transform:{translation:{x: 0.0, y: 0.0, z: -0.09670}, rotation:{x: 0.7071068, y: 0.7071068, z: 0.0, w: 0.0}}}}"
```

Output should be:

```bash
joint_state:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs:         0
    frame_id: ''
  name:
    - outer_yaw
    - outer_pitch
    - outer_insertion
    - outer_roll
    - outer_wrist_pitch
    - outer_wrist_yaw
  position: [0.0, -0.0, 0.093, 0.0, -0.0, 0.0]
  velocity: []
  effort: []
```

## Launch dvrk_teleop_node

Launch the default keyboard and PSM2 teleop:

```
rosrun dvrk_planning_ros dvrk_teleop_node.py -y config/keyboard_psm2.yaml
```

Then in another terminal, run the keyboard node:

```
rosrun dvrk_planning_ros psm_teleop_keyboard.py
```

To specify another config file path, and/or another package where the config is relative to:

```
rosrun dvrk_planning_ros dvrk_teleop_node.py -p <another_package> -y <relative_path_to_yaml>
```
