SLIDERAIL Weld Cell Simulator
This repository contains a robotic sliderail weld-cell simulator built in Webots, with Python controllers for robot motion, waypoint handling, and end-effector logic. The project is intended as a portable, clone-and-run simulation that demonstrates robotic programming, kinematics, and manufacturing automation concepts.
The simulator was developed as part of a no-code / low-code robotic programming environment, where robot behavior is driven by world objects (waypoints, safe points, pick points, and release points) rather than hard-coded trajectories.

What This Project Demonstrates
    • Industrial robotic pick-and-place operation for a sliderail weld cell (material handling only)
    • Waypoint-driven robot motion using world objects
    • Simulated magnetic end effector (on/off control)
    • Safe-position based motion sequencing (approach / depart)
    • Inverse kinematics using a URDF model
    • Python-based robot and world controllers in Webots

Requirements
Software
    • Webots (tested with R2025a)
        ? Download: https://cyberbotics.com
    • Python 3.x (configured inside Webots)
Python Packages
Install the following Python dependencies using the Python interpreter configured in Webots:
numpy
scipy
ikpy
pynput
These are listed in requirements.txt.
Note: Modules such as controller, os, pathlib, time, etc. are part of Python/Webots and do not need to be installed with pip.

How the Simulation Works
This simulation represents the pick-and-place (material handling) portion of a sliderail robotic weld cell. No welding logic is implemented; the focus is on part transfer and motion sequencing.
Motion Logic
    1. The robot begins at its home position.
    2. The robot moves to an initial safe position.
    3. It moves to the pick position.
    4. At a yellow waypoint, the simulated magnet turns ON.
    5. The robot picks up a simulated crossmember.
    6. The robot moves to a different safe position.
    7. It moves to a safe position above the drop location.
    8. The robot moves down to the drop position.
    9. At a green waypoint, the simulated magnet turns OFF.
    10. The crossmember is released and drops to the floor just before the end effector moves away.
    11. The robot returns to a safe position and continues the remainder of the cycle.
User Interaction
    • The simulation is controlled via keyboard input.
    • Press r to start the run cycle.
    • The world/robot controller handles state transitions and sequencing.

Setup Instructions
    1. Install Webots
        ? https://cyberbotics.com
    2. Clone this repository
    • git clone <repo-url>
    3. Open the world file
        ? Launch Webots
        ? Open:
        ? worlds/WORLD_fixed.wbt
    4. Install Python dependencies In a command prompt, using the same Python that Webots uses:
    • pip install -r requirements.txt
    • You can verify the Python interpreter Webots uses via:
        ? Tools ? Preferences ? Python command
    5. Run the simulation
        ? Press Run in Webots
        ? Press r to begin the pick-and-place cycle
Note: This demo implements pick-and-place logic only. Welding behavior is not simulated.

Project Structure
SLIDERAIL-WELD-CELL/
?? controllers/
?  ?? robot_controller2/
?  ?  ?? robot_controller2.py
?  ?? World_Controller/
?     ?? World_Controller1.py
?
?? meshes/
?  ?? *.STL
?
?? urdf/
?  ?? ROBOT_A.urdf
?
?? worlds/
?  ?? WORLD_fixed.wbt
?
?? requirements.txt
?? README.md
All mesh and URDF paths are relative, so the project can be cloned into any directory and run without modification.

Notes on Portability
    • Mesh paths are relative (no machine-specific absolute paths)
    • URDF paths are resolved dynamically using pathlib
    • Python dependencies are explicitly listed
If something fails to load, check the Webots Console for missing-dependency or path warnings.

Scope and Limitations
This demo is intentionally scoped to pick-and-place logic only.
    • The simulation focuses on motion sequencing, safe-position logic, waypoint control, and end-effector state management.
    • Process optimization, fixture design, placement accuracy, and welding operations are explicitly out of scope.
    • The simulated crossmember is released at the drop location without precise placement to emphasize conceptual workflow and control logic, not final manufacturing readiness.
This approach reflects early-stage automation development, where behavior and sequencing are validated before process optimization.

Intended Audience
This project is intended for:
License / Usage
This project is provided for demonstration and educational purposes. Adaptation and extension are encouraged.

Author
Juan Rebollo
Robotics • Automation • Simulation
