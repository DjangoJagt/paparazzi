# Autonomous Flight of Micro Air Vehicles Group 11

## Overview
This repository houses the custom module for the Paparazzi UAV system, crafted for the Autonomous Flight Challenge. Our primary aim was to showcase the advanced autonomous navigation and obstacle avoidance capabilities of micro air vehicles.

## Key Components
The project utilizes the Paparazzi UAV system, augmented with custom modules for improved functionality. The critical components developed for this endeavor include:

- **GROUP_11 Module:** The primary module, integrating advanced algorithms for autonomous navigation, obstacle avoidance, and interaction with the environment. It employs real-time visual processing by interfacing closely with the onboard camera system.
- **cv_detect_color_object Module:** This module is crucial for identifying specific colors within the camera's visual feed, enabling the UAV to recognize obstacles and designated targets.
- **ground_detection Module:** Facilitates ground detection, a key aspect of ensuring safe autonomous flight operations.

## Usage
To deploy this project:
1. Clone the repository into your local Paparazzi UAV environment.
2. Select aircraft `GROUP_11` 

This project is initially configured for Bebop drones but is adaptable for other UAVs supported by Paparazzi.

## Competition Code
The competition-specific files reside in the `GROUP_11` directory and main ones are as follows:
- `conf/modules/GROUP11.xml`
- `conf/airframes/tudelft/bebop_GROUP_11.xml`
- `sw/airborne/modules/GROUP_11` includes the c and header file
- `conf/flight_plans/tudelft/course_GROUP_11.xml`

The `sw/airborne/modules/computer_vision/cv_detect_color_object` and `sw/airborne/modules/ground_detection` modules were pivotal to our autonomous flight challenge configuration, providing essential functionalities.

## Testing and Development
The project's development involved various files and configurations not strictly part of the mentioned modules. These were essential during the testing phase to refine our algorithms and system capabilities. Key insights from our development process might offer valuable perspectives for those looking to replicate or further innovate on this work.

### Initial Testing with Orange Avoider Guided Module
In the early stages of testing, we utilized the `orange_avoider_guided` module as a baseline for our navigation and obstacle avoidance algorithms. This preliminary phase allowed us to identify crucial adjustments and enhancements necessary for our custom module development, setting a foundation for more sophisticated functionalities.

### Optic Flow Testing
Within the `OPTICFLOW_TESTING` folder, we have included test files intended for exploring the potential of optic flow in enhancing UAV navigation. The files contain modified versions of the existing optical flow calculators from the `sw/airborne/modules/computer_vision/` folder. The code was modified to make it fit for our purpose of navigation. Despite the promising aspects of optic flow for real-time environmental interaction and obstacle detection, it was ultimately not incorporated into our competition setup. These tests, however, represent an area of potential future exploration for the project.
