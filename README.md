# MTE-Robotics-Lab
This repository is the codebase for the lab submission for the MTE Robotics Lab Mini-Project by Group 4.

# Problem Statement
Perform pick-and-place operation-based sorting of coloured boxes on a table into bins. A camera mounted above the table locates the boxes, and a UR5 Robotic Manipulator sorts the boxes by colour into their corresponding bins, located around the robot. Simulate this setup using ROS and any essential setups/libraries/functions as per requirement.
# Methodology
## Libraries and Functions Used
The following ROS integrations and setups were used (in order of appearance):
### Gazebo
Gazebo is the primary simulation environment in ROS to integrate robots and environmental assets together, to observe their behaviour. By creating environments pertaining to specific use cases for a robot (for example, city roads for self-driving cars or a factory for an industrial robot), it is possible to observe how the robot autonomously interacts with its environment. 
  
The Gazebo world used in this submission was custom-built to fit the problem statement.
### MoveIt! Motion Planning and RViz Robot Visualisation
MoveIt! is a library of functions that enables autonomous motion-planning and other interactive tasks with robots using ROS. Coupled with the RViz Robot Visualisation setup, it is a powerful tool to enable robots to perform autonomous tasks and directly observe the results visually.

This submission uses the default integration for MoveIt! and RViz. The MoveIt! libraries were tweaked to fit a UR5 Robotic Manipulator, while the RViz Planning Scene was custom-built to fit the problem statement.
### UR5 Robotic Manipulator and URDF
The UR5 is a Robotic Manipulator part of the family of Cobots by Universal Robots. Cobots are robots built specifically for collaborative tasks with human operators or preforming tasks in environments in close proimity with humans. Salient features of cobots include a high emphasis on safety and collaborative functions, often working alongside a human on the same object in factories.

To simulate the UR5, the Universal Robot Descriptive Format (URDF), an XML-based description of the links, joints and other essential parameters of a robotic manipulator is used in conjunction with the integrations above to visualise and control the robot.

All essential assets for the UR5 used in this submission was obtained from the [official Universal Robots repository](https://github.com/ros-industrial/universal_robot).
## Building the Gazebo World and the RViz Planning Scene
### Gazebo World
The Gazebo world contains the following objects:
* A UR5 Robotic Manipulator mounted on a pedestal;
* A table on which the boxes are kept;
* A logical camera mounted above the table with the table and any objects on it in it's field of view; and
* 3 bins, coloured red, blue and green respectively.

The table also contains 6 boxes (2 red, 2 green, 2 blue) arranged randomly in front of the UR5. The boxes are individually spawned using a script to preserve their frames.
### RViz Planning Scene
The RViz Planning Scene contains the following objects:
* A UR5 Robotic Manipulator mounted on a pedestal;
* A placeholder object for the top of the table as seen in the Gazebo world; and
* 6 objects representing the boxes as seen in the Gazebo world.

The bins and logical camera were deemed non-essential to motion-planning requirements through observation, and have thus not been included.
## Setting up TF
TF ('TransForm') is the primary method for objects as detected through the logical camera to be visualised as frames and acted upon in and using RViz. The logical camera visualises the frames of the boxes in the RViz Planning Scene; this information can then be used to direct the UR5 to travel to the specified pose (the position of the specified box) for executing the pick operation.
## ROS Scripting
The ROS Scripting oversees the whole setup and manages the interconnectivity of the various communicating nodes in the setup. ROS Scripting covers the following aspects in this submission:
* Spawning the Gazebo, MoveIt!, RViz setups together through a ROSLaunch file;
* Handling the UR5 node, the primary node used to control the UR5 used in this task; and
* Handling TF data from the logical camera and feeding the corresponding actions to the UR5.
