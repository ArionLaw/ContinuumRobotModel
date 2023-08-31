
## Hamlyn Contributions (v0):
* Python
* Lightweight modular kinematics -> Can be used in simulation
* Controller with well defined output/input interfaces ->
  * Seamless and robust switch between simulation and real
* Verified kinematics ini URDF and solver -> Simulation/real kinematics verification
* Minimal interfaces to OutputModality -> OutputModality can have many hardware dependencies depending on robot

## Newer Contributions (v1):
### Relative to CISST and CRTK:
* WebGUI visualizer to autogenerate URDF and kinematics solver
* Strong Abstraction of WorkcellDescription vs Controller ->
  * All information about the robot and environment in Workcell -> Visualization
* Controller manager: Safety guarantee of switching controllers.
* Strong abstraction/concept of a Controller:
  * Modular and allows to implement only a subset of needed controllers (Not every OutputModality and InputDevice combination needs all the controller interface in CRTK). The CRTK requires every robot hardware/simulation to have all 10+ interfaces.
  * Allows user to implement their own controller.
* Abstract C++ Interface for the controller which is ROS independent. ROS is one implementation of the interface
  * Allows special instantiation of C++ MTM->PSM (1000hz haptic feedback)
  * ROS is the default interface for beginner users. Some of the community frowns upon ROS, but we need to prove them wrong that it is stable these days. Even better with ROS2.
* Collision checking in Motion Planning
* URDF as the base kinematics which is easily readable, with tutorials, visualizers, export to unity, etc

### Relative to Motion planning frameworks (Moveit/Tesseract/OpenRave):
* Focus on collaborative robotics (dVRK/UR/Franka)
* Medical robotics teleoperation capabilities (Vision orientation control, master slave follower controller, haptics, scaling)
* Human in the loop controller centric
* Has latest virtual fixtures capabilities from cisst and max's anatomical mesh

## Other SCRTK Traits (v1):
* C++ for underlying things, python for non-time critical
* Lightweight (in a sense, only build what you need, but can be heavy if build everything)
* Has latest motion planning libraries


## Implementation Plan

Prove of concepts to test:
* Constrained Optimization and virtual fixtures

Implementation Plan for June 15th:
* GeneratorWebGUI
* WorkcellDescription
  * URDF Importer
  * Robot Kinematics and Joint limits
  * Virtual fixtures
* VisualizationRViz
* TeleopController
* ConstrainedOptimization
* CartesianController
* WatchDog
