roscore

qlacommand -c close-relays
qlacommand -c open-relays

roscd dvrk_config
rosrun dvrk_robot dvrk_console_json -j hsc-dVRK/custom-tools/console-PSM2-Fetal.json 

cd ~/ContinuumRobotModel
source devel/setup.bash
rosrun dvrk_planning_ros dvrk_teleop_node.py -y config/keyboard_psm2_Fetal.yaml

cd ~/ContinuumRobotModel
source devel/setup.bash
rosrun dvrk_planning_ros psm_teleop_keyboard.py

rostopic echo /PSM2/servo_jp

