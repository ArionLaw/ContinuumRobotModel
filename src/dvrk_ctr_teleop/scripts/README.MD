"""Change to the directory
roscd dvrk_config
cd hsc-dVRK

run qlacommand -c close-relays (hear clicking)

Start the dvrk console
rosrun dvrk_robot dvrk_console_json -j console-MTML-MTMR.json
Make sure you power on and home

Start the AMBF simulation
cd 0811MTMtest/

ambf_simulator --launch_file launch.yaml -l 1,4,16,21,26,52,53

in ContinuumRobot folder 
python3 src/dvrk_ctr_teleop/scripts/ambf_crtk_translator.py

rosrun dvrk_planning_ros dvrk_teleop_node.py -p dvrk_ctr_teleop -y config/mtml_psm1_mtmr_psm2_cam_sim.yaml

for keyboard
python3 src/dvrk_planning/dvrk_python_v1/dvrk_planning_ros/scripts/psm_teleop_keyboard.py

"""
