# ContinuumRobotModel

TODO: This readme could be in src/dvrk_ctr_teleop

## Build

```bash
cd ContinuumRobotModel
catkin build
```

## Running the controller

Terminal 1:
```bash
roscore
```

Terminal 2:
```bash
qlacommand -c close-relays
qlacommand -c open-relays
```

Terminal 3:
```bash
roscd dvrk_config/
cd hsc-dVRK
### for moving psm arms
rosrun dvrk_robot dvrk_console_json -j console-SUJ-ECM.json 
### for running teleop
rosrun dvrk_robot dvrk_console_json -j custom-tools/console-MTMR-PSM2-Fetal.json
```
In console:
Under Arms/PSM2 tab, choose Tool: Peter Francis Design with da Vinci Classic (AT BOTTOM)
('Power On', 'Home', Check 'Direct Control' under 'PSM2 PID' tab)

If running teleop, continue: 
Terminal 4:
```bash
source devel/setup.bash
# Keyboard control: 
rosrun dvrk_planning_ros dvrk_teleop_node.py -p dvrk_ctr_teleop -y config/keyboard_psm2_fetal.yaml
# OR
# Mtm control: 
rosrun dvrk_planning_ros dvrk_teleop_node.py -p dvrk_ctr_teleop -y config/mtmr_psm2_fetal.yaml
# OR
# Both mtm and psm control
rosrun dvrk_planning_ros dvrk_teleop_node.py -p dvrk_ctr_teleop -y config/mtml_psm_mtmr_psm2_fetal.yaml
```

For keyboard only, terminal 5:
```bash
source devel/setup.bash
rosrun dvrk_planning_ros psm_teleop_keyboard.py
```

## Setting up the camera perspective control config

[TODO] Diagram

### Setup the HSRV frame relative to the MTM base

In your dvrk_teleop config, (e.g. dvrk_planning_ros/config):
```yaml
    input:
      # ... 
      input_2_input_reference_rot:
        quaternion:
          x: 0
          y: 0
          z: 0
          w: 1
```


### Setup Endoscope camera frame relative to the PSM base
Run SUJ and PSM1 and PSM2
Terminal 1:
```bash
roscore
```

Terminal 2:
```bash
roscd dvrk_config/
cd hsc-dVRK
rosrun dvrk_robot dvrk_console_json -j console-SUJ-ECM.json 
```

Open Rviz, Terminal 3:

```bash
source devel/setup.bash
roslaunch dvrk_ctr_teleop suj_ecm_rviz.launch
```

Now move the setup joint arms to your desired location.
Find the parent and child you are interested in, and run the tf obtainer with parent and chile arguments:

```bash
source devel/setup.bash
rosrun dvrk_planning_ros generate_tf_mat.py PSM2_base ECM
```

Then put in your yaml file for the corresponding controller
```yaml
    output:
      # ... 
      output_2_output_reference_rot:
        quaternion:
          x:  -0.2610866462261644
          y:  0.954154567645924
          z:  0.13158294821796376
          w:  0.06409954712056945
```


