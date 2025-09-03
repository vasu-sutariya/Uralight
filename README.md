<div align="center">

<h2>URAlight</h2>

<img src="./uralight.gif" alt="URAlight demo" />

</div>

Unity-based software for programming and simulating manipulator robots. URAlight focuses on fast environment building, safe motion planning with collision avoidance, and an extendable UI for custom workflows.



### Key Features

- **Core**
  - Library of Universal Robots (UR) models
  - Control multiple robots simultaneously
  - Build environments: import and place 3D objects in the scene
  - Polyscope-like programs available in-app
  - Joystick-style controls for quick jogging
  - Run and create programs from the app or upload a Python script
  - Python API to drive robots and the scene programmatically
  - Create custom UI elements to use functions from scripts
 

- **Technical**
  - Motion types: Joint space (MoveJ) and Cartesian space (MoveL)
  - Timing and smoothing: blend radius, time constraint, max velocity and acceleration
  - Collision avoidance: grid-based and sampling-based planners
  - Object interactions: attach/detach objects to the end-effector or other objects
  - Networking: socket communication with UR controllers

### Project Structure

- `Assets/Added files/scripts/`: core gameplay and tooling scripts (camera, jog, CAD import, etc.)
- `Assets/Added files/ROBOT Models/`: robot assets, kinematics, and related scripts

### Requirements

- **Unity**: 6000.0.34f1 (open with this exact version for best compatibility)
- **OS**: Windows 10 or later (tested)
- **Optional**:
  - Network access to a UR controller if you plan to connect to a real robot
  - Python runtime if you intend to upload/execute Python scripts from the app



### Basic Usage

- **Add robots**: Use the library to place UR robots in the scene. Multiple robots are supported.
- **Jogging**: Use the joystick-style UI to move the robot quickly for setup.
- **Programming**: Create and run programs from within the app or upload a Python script.
- **Environment**: Import 3D models (e.g., OBJ) and position them to build your workcell.
- **Collision safety**: Enable collision avoidance in the planner options when executing motions.

### Python API

- Control robots, send motions (MoveJ/MoveL), and manipulate the scene programmatically.
- Communicates with the app over sockets for command execution.
- Ensure a Python runtime is installed and reachable from your environment.


### Download app

Grab packaged builds from the Releases page:
[https://github.com/vasu-sutariya/Uralight/releases]

### Project Status

- This project is in a very early stage and may contain many bugs.

- **Currently available**
  - In-app program generation
  - Simple motions: MoveJ and MoveL
  - UR10 robot model
  - CAD import (e.g., OBJ)

- **Under development**
  - Blend radius trajectories
  - Collision avoidance
  - Custom UI system
  - Python API
  - Safe connection to real robots
  - Scene save/load
  - Multiple configuration support
  - Other brand robt models
  - Physics integration for Force related applications
