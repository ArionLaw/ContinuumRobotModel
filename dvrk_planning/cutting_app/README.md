# Installation

Install toppra:
```
pip install toppra
```

# Usage

Tab 4: Allows user to enter movements commands for the robot into the terminal. The robot can move and rotate along the x, y, and z axes.

Tab 6: Allows user to specify points along a path to save the current tip position as a waypoint by pressing "w" in the terminal. You can keep moving the robot using Tab 4, and saving points using this tab, then finally send the waypoints as goals for the robot to move through

Tab 7: Allows user to modify the current trajectory that the robot is moving in by entering directional input into a pygame window. NOTE: Look for a black input box titled "Click Here and Press Keys to Teleop". Also select teleop/motion generator through pygame. Allows user to select either teleop or motion generator functionality through the terminal

# Running

Tab 1: 
```
cd sim_ws/ambf/bin/lin-x86_64/
./ambf_simulator -l 5
```

Tab 2:
``` 
cd dvrk_ws/src/dvrk_planning/ambf_ros_crtk/scripts/
source ~/sim_ws/ambf/build/devel/setup.bash
python3 ambf_crtk_translator.py 
```
Note: May need to run twice.

Tab 3:
```
cd dvrk_ws
source devel/setup.bash
rosrun dvrk_planning_ros dvrk_teleop_node.py -p cutting_app -y config/keyboard_psm2_rts420007.yaml
```

Tab 4:
```
cd dvrk_ws
source devel/setup.bash
rosrun dvrk_planning_ros psm_teleop_keyboard.py
```

Tab 5:
```
cd dvrk_ws
source devel/setup.bash 
rosrun dvrk_planning_ros motion_generator.py -p cutting_app -y config/motion_generator.yaml
```

Tab 6:
```
cd dvrk_ws
source devel/setup.bash 
rosrun cutting_app teleop_goal_interface.py 
```

Tab 7:
```
cd dvrk_ws
source devel/setup.bash
rosrun dvrk_planning_ros shared_autonomy_control.py 
```


RVIZ

Tab 1:
```
cd dvrk_ws
source devel/setup.bash 
roslaunch dvrk_description psm_rviz.launch 
```

Tab 2:
```
cd dvrk_ws
source devel/setup.bash 
rosrun dvrk_planning_ros rviz_interface.py 
```

DVRK

Tab 1:
```
source /opt/ros/noetic/setup.bash
roscore 
```

Tab 2:
```
source /opt/ros/noetic/setup.bash
cd dvrk_ws
source devel/setup.bash 
roscd dvrk_config
rosrun dvrk_robot dvrk_console_json -j hsc-dVRK/console-PSM2.json
```

Tab 3:
```
cd dplan_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd src
rosrun dvrk_planning_ros dvrk_teleop_node.py -y config/keyboard_psm2.yaml
```

Tab 4:
```
cd dplan_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun dvrk_planning_ros psm_teleop_keyboard.py
```

Tab 5:
```
cd dplan_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash 
rosrun dvrk_planning_ros motion_generator.py -p cutting_app -y config/motion_generator.yaml
```

Tab 6:
```
cd dplan_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash 
rosrun cutting_app teleop_goal_interface.py 
```

Tab 7:
```
cd dplan_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun dvrk_planning_ros shared_autonomy_control.py 
```

RVIZ

Tab 1:
```
cd dplan_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash 
roslaunch dvrk_description psm_rviz.launch 
```

Tab 2:
```
cd dplan_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash 
rosrun dvrk_planning_ros rviz_interface.py 
```

