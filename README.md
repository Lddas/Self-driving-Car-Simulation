### Autonomous Car Project 

## Overview

This project was developed as part of an academic course at TECNICO LISBON. The goal was to design and implement an autonomous car capable of traveling from one point to another on the TECNICO LISBON campus, respecting realistic car trajectories. The system uses path planning algorithms, control systems, and realistic kinematic modeling to ensure the car follows a feasible trajectory.

Features

	•	Realistic Trajectory Planning: Implements a path planning algorithm that respects the kinematic constraints of the car.
	•	Autonomous Navigation: The car navigates autonomously between predefined start and destination points on the campus.
	•	Kinematic Modeling: Includes a kinematic model for precise motion simulation, accounting for car length, turning radius, and velocity constraints.
	•	Visualization: Uses matplotlib to visualize the car’s path in real time, showing both reference and actual trajectories.

Project Structure

	•	main.py: Main script to run the simulation, including path planning and car control.
	•	code_leo.py: Contains specific functions used for reference trajectory generation.
	•	path_planning.py: Includes the path planning algorithm that computes the optimal path from the starting point to the destination.
	•	render.py: Handles the graphical rendering of the car’s motion using matplotlib and cv2.
	•	robot.py: Defines the car’s kinematics, control, and behavior for the autonomous navigation task.

Installation

To run this project, you will need:

	•	Matplotlib
	•	OpenCV
	•	Numpy

How to Run

	1.	Modify the STARTING_POINT and DESTINATION variables in main.py to set the desired starting and destination points on the campus.
	2.	Run the simulation

 Project Details

	•	Start and Destination Points: The project uses preset coordinates representing locations on the TECNICO LISBON campus. These can be modified based on the desired scenario.
	•	Control Parameters: The simulation uses a proportional controller to adjust the car’s path in response to deviations from the reference trajectory.
	•	Visualization: A 2D plot shows the car’s movement, with the actual and reference paths displayed for comparison.

Future Improvements

	•	Add obstacle detection and avoidance.
	•	Implement more advanced control strategies for better handling of complex environments.
	•	Integrate a 3D visualization for a more realistic simulation.
