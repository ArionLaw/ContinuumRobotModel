# Description

A translator from ambf messages to crtk messages ... servo_jp, jaw/servo_jp, measured_js, jaw/measured_js

# Instructions

If not done before, compile ambf https://github.com/WPI-AIM/ambf

```
source <path_to_ambf>/build/devel/setup.bash

cd src/dvrk_ctr_teleop/scripts

python3 ambf_crtk_translator.py


# Launching Simulator
cd 0811MTMtest
ambf_simulator --launch_file launch.yaml -l 1,4,16,21,26,50,51


# Launch Teleop
rosrun dvrk_planning_ros dvrk_teleop_node.py -p dvrk_ctr_teleop -y config/mtml_psm1_mtmr_psm2_cam.yaml


# Debugging 
roslaunch dvrk_ctr_teleop debug_tf.launch 

'''