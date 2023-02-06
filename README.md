# InterIIT MidPrep: Robotic Charging Challenge
- CAD and Analysis
- Planning
- CV
- Simulation

## CAD and Analysis
This folder has the main assembly file named `Solid model/jlr_robotic_charging_station.STEP` . The FEM results are included in `FEMSimulation(Ansys)` folder. The BOM and the supporting docs are available in `bill of materials`.

## Planning
- The folder `Robot-Arm-Control` contains MATLAB files used for path planning and robot arm characterization.
- `main.m` is used to initialize the robot DH parameters and collision boxes.
- `/RobotKinematics` contains files for forward and inverse kinematics, collision detection and transformation matrices.
- `/planning` contains files for joint space planning, energy computation and velocity planning.
- `/cartesian_straight_line` contains files used for straight line path in cartesian coordinates.
- `/torque_files` contains files used for computing torques for any configuration and kinematics.
- `/plotting` contains all the scripts used for generating plots.

## CV
 - `/CV.py` It is a function which takes a loaded model, frame of the video and outputs coordinates of 4 points wrt to camera frame that has to be preloaded.
 - Monocular camera trajecrtory obtaining depths of 4 key point in mm
 - `/Stereo` and `/Monocular` contains all the files for location estimation for circle, straight line and parallel using stereo and monocular estimations respectively.



## Simulation
- `/Simscape Simulation` contains all the models and the dependent files for simulating the trajectory of the robot arm from any initial point to final point.
- In order to initiate the simulation, update the initial and final desired positions in `waypointGeneration.m` file. After running this script, run the `finalSimscapeModel.slx` model.
- The remaining files and folders contains the supporting files and models for running the Simscape model.
- The simulink model also launches the MATLAB 3D world, which was used for visualization and generating images for training the YOLO detection model.