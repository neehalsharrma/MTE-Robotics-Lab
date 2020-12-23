#! /usr/bin/env python

import rospy, geometry_msgs.msg, os
from hrwros_gazebo.msg import LogicalCameraImage
from math import radians
from rospy.exceptions import ROSInterruptException
from lib import UR5MoveIt

def env_data():
    '''
    Data of all environment-specific parameters:
    1. Vacuum Gripper Width
    2. Box Size

    Returns:
        All environment-specific data.
    '''
    box_length = 0.15  # Length of the box
    vacuum_gripper_width = 0.117  # Vacuum Gripper Width

    # Return data when called
    return [box_length,
            vacuum_gripper_width]

def joint_angles_data():
    '''
    Data of all joint angles required for various known positions:
    1. Home: Ready-position for picking objects off the conveyor
    2. Red Bin: Red Bin to place Red packages
    3. Green Bin: Green Bin to place Green packages
    4. Blue Bin: Blue Bin to place Blue packages
    
    Returns: 
        Joint angles for all positions when called.
    '''
    # Home/Ready angles for picking
    home_joint_angles = [radians(180),
                        radians(-75),
                        radians(110),
                        radians(-125),
                        radians(-90),
                        radians(0)]

    # Red Bin angles
    red_bin_joint_angles = [radians(65),
                           radians(-55),
                           radians(80),
                           radians(-115),
                           radians(-90),
                           radians(0)]

    # Green bin angles
    green_bin_joint_angles = [radians(0),
                             radians(-55),
                             radians(80),
                             radians(-115),
                             radians(-90),
                             radians(0)]

    # Blue bin angles
    blue_bin_joint_angles = [radians(-95),
                            radians(-55),
                            radians(80),
                            radians(-115),
                            radians(-90),
                            radians(0)]

    # Return data when called
    return [home_joint_angles, 
            red_bin_joint_angles, 
            green_bin_joint_angles,
            blue_bin_joint_angles]

def subscriber_init():
    pass

def controller():
    pass

if __name__ == "__main__":
    '''
    Controls overall execution
    '''

    ur5 = UR5MoveIt()

    # Obtain prerequisite data
    joint_angles = joint_angles_data()
    env_values = env_data()

    # Start execution
    controller()