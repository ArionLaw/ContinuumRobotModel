# ContinuumRobotModel

rosrun dvrk_planning_ros dvrk_teleop_node.py -y config/keyboard_psm2_Fetal.yaml

# Terminal 1 
roscore

# Terminal 2
qlacommand -c close-relays

qlacommand -c open-relays

# Terminal 3 
roscd dvrk_config/

cd hsc-dVRK/
### for moving psm arms
rosrun dvrk_robot dvrk_console_json -j console-SUJ-ECM.json 
### for running teleop
rosrun dvrk_robot dvrk_console_json -j console-PSM2.json

('Power On', 'Home', Check 'Direct Control' under 'PSM2 PID' tab)

# Terminal 4 
cd ContinuumRobotModel/

source devel/setup.bash

rosrun dvrk_planning_ros psm_teleop_keyboard.py

# Terminal 5
cd ContinuumRobotModel/

source devel/setup.bash

rosrun dvrk_planning_ros dvrk_teleop_node.py -y config/keyboard_psm2_Fetal.yaml

