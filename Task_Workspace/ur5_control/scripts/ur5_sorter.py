#! /usr/bin/env python
'''
ROS Script to sort Red, Green and Blue objects on a table into bins.
'''

import rospy
import geometry_msgs.msg
import os
from env_sim.msg import LogicalCameraImage
from math import radians
from rospy.exceptions import ROSInterruptException
from lib import UR5MoveIt

MasterString = ''  # String to store model names from camera_callback()
ReadyFlag = True  # Flag to indicate ready state of robot


def env_data():
    '''
    Data of all environment-specific parameters:
    1. Vacuum Gripper Width
    2. Box Size

    Returns:
        (list) All environment-specific data.
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
        (list) Joint angles for all positions when called.
    '''
    # Home/Ready angles for picking
    home_joint_angles = [radians(0),
                         radians(-90),
                         radians(-90),
                         radians(-90),
                         radians(90),
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


def camera_callback(msg_camera):
    '''
    Callback function for Conveyor Logical Camera Subscriber.

    Parameters:
        msg_camera (LogicalCameraImage object): Data about all the objects detected by the Logical Camera.
    '''
    global MasterString, ReadyFlag

    if(ReadyFlag):
        for i in range(0, len(msg_camera.models)):
            MasterString = MasterString + ' ' + msg_camera.models[i].type
        model_list = MasterString.split(' ')

        for i in model_list:
            if('package' in i):
                MasterString = i
                break

    else:
        MasterString = ''


def pose_set(trans, rot):
    '''
    Assigns pose values w.r.t the world-frame to a PoseStamped object.

    Parameters:
        trans (float[]): Translation Values.
        rot (float[]): RPY Rotation Values.

    Returns:
        pose (geometry_msgs.msg.PoseStamped() object): The complete pose with values.
    '''
    # If you want to override any specific values received, use this
    override = [trans, [-0.5, -0.5, 0.5, 0.5]]
    if(override != []):
        trans = override[0]
        rot = override[1]
        print(override)

    # Generating the object
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = trans[0]
    pose.pose.position.y = trans[1]
    pose.pose.position.z = trans[2]
    pose.pose.orientation.x = rot[0]
    pose.pose.orientation.y = rot[1]
    pose.pose.orientation.z = rot[2]
    pose.pose.orientation.w = rot[3]

    return pose


def box_plan(box_name, box_length, vacuum_gripper_width):
    '''
    Pick-planning for the boxes.

    Parameters:
        box_name (str): The colour of the box detected.
        box_length (float): The size of the box.
        vacuum_gripper_width (float): The width of the vacuum gripper.
    '''
    # Offset for end effector placement
    delta = vacuum_gripper_width + (box_length/2)
    try:
        if('Red' in box_name):  # Red box detected
            # Obtaining the TF transform of the box
            if('1' in box_name):
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_r1_frame",
                                                                       rospy.Time(0))
            else:
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_r2_frame",
                                                                       rospy.Time(0))
            # Execute pick operation
            box_pose = pose_set(box_trans, box_rot)  # Collating pose values
            box_pose.pose.position.z = box_pose.pose.position.z + delta  # Adding Z Offset
            ur5.go_to_pose(box_pose)
            # Activate Vacuum Gripper
            os.system(
                'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            # Add the Red box to the planning scene
            box_pose.pose.position.z = box_pose.pose.position.z - delta  # Removing Z Offset
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            # Log the operation
            rospy.loginfo(
                '\033[91m' + "Red Package Picked!" + '\033[0m')
        elif('Green' in box_name):  # Green box detected
            if('1' in box_name):
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_g1_frame",
                                                                       rospy.Time(0))
            else:
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_g2_frame",
                                                                       rospy.Time(0))

            box_pose = pose_set(box_trans, box_rot)
            box_pose.pose.position.z = box_pose.pose.position.z + delta
            ur5.go_to_pose(box_pose)
            os.system(
                'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            box_pose.pose.position.z = box_pose.pose.position.z - delta
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            rospy.loginfo(
                '\033[92m' + "Green Package Picked!" + '\033[0m')
        elif('Blue' in box_name):  # Blue box detected
            if('1' in box_name):
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_b1_frame",
                                                                       rospy.Time(0))
            else:
                (box_trans, box_rot) = ur5.tf_listener.lookupTransform("/world",
                                                                       "/logical_camera_2_package_b2_frame",
                                                                       rospy.Time(0))

            box_pose = pose_set(box_trans, box_rot)
            box_pose.pose.position.z = box_pose.pose.position.z + delta
            ur5.go_to_pose(box_pose)
            os.system(
                'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: true"\n')
            box_pose.pose.position.z = box_pose.pose.position.z - delta
            ur5.add_box(box_name, box_length, box_pose)
            ur5.attach_box(box_name)
            rospy.loginfo(
                '\033[94m' + "Blue Package Picked!" + '\033[0m')
    except:
        pass


def bin_plan(box_name, bin_name, bin_joint_angles):
    '''
    Place-planning for the bins.

    Parameters:
        bin_name (str): The colour of the bin.
        bin_joint_angles (float[]): The joint anglenv_values[0], env_values[1]es of the required bin.
    '''
    if(bin_name == 'R'):  # Red bin
        # Set joint angles for the bin
        ur5.set_joint_angles(bin_joint_angles)
        # Deactivate the Gripper
        os.system(
            'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        # Remove the box from the planning scene
        ur5.detach_box(box_name)
        ur5.remove_box(box_name)
        # Log the operation
        rospy.loginfo(
            '\033[91m' + "Red Package Placed!" + '\033[0m')
    elif(bin_name == 'G'):  # Green bin
        ur5.set_joint_angles(bin_joint_angles)
        os.system(
            'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        ur5.detach_box(box_name)
        ur5.remove_box(box_name)
        rospy.loginfo(
            '\033[92m' + "Green Package Placed!" + '\033[0m')
    elif(bin_name == 'B'):  # Blue bin
        ur5.set_joint_angles(bin_joint_angles)
        os.system(
            'rosservice call /mte_roblab/ur5_1/activate_vacuum_gripper "activate_vacuum_gripper: false"\n')
        ur5.detach_box(box_name)
        ur5.remove_box(box_name)
        rospy.loginfo(
            '\033[94m' + "Blue Package Placed!" + '\033[0m')


def subscriber_init():
    '''
    Definitions and setups of all Subscribers.
    '''
    # Subscriber for Table Logical Camera
    rospy.Subscriber('/mte_roblab/logical_camera_2',
                     LogicalCameraImage,
                     camera_callback)


def controller():
    '''
    Executes the main operations.
    '''
    global MasterString, ReadyFlag

    # Go to a home position in preparation for picking up the packages
    ur5.set_joint_angles(joint_angles[0])

    # Execute box-bin planning
    while(not rospy.is_shutdown()):
        if('package_r' in MasterString):  # Detecting Red package
            ReadyFlag = False  # Operating
            if('1' in MasterString):  # Detecting the specific model
                # Pick operation
                box_plan('Red Box 1', env_values[0], env_values[1])
                bin_plan('Red Box 1', 'R', joint_angles[1])  # Place operation
            else:
                # Pick operation
                box_plan('Red Box 2', env_values[0], env_values[1])
                bin_plan('Red Box 2', 'R', joint_angles[1])  # Place operation
            ReadyFlag = True  # Ready for next operation
        elif('package_g' in MasterString):
            ReadyFlag = False
            if('1' in MasterString):
                box_plan('Green Box 1', env_values[0], env_values[1])
                bin_plan('Green Box 1', 'G', joint_angles[2])
            else:
                box_plan('Green Box 2', env_values[0], env_values[1])
                bin_plan('Green Box 2', 'G', joint_angles[2])
            ReadyFlag = True
        elif('package_b' in MasterString):
            ReadyFlag = False
            if('1' in MasterString):
                box_plan('Blue Box 1', env_values[0], env_values[1])
                bin_plan('Blue Box 1', 'B', joint_angles[3])
            else:
                box_plan('Blue Box 2', env_values[0], env_values[1])
                bin_plan('Blue Box 2', 'B', joint_angles[3])
            ReadyFlag = True

        ur5.set_joint_angles(joint_angles[0])


if __name__ == "__main__":
    '''
    Controls overall execution
    '''
    ur5 = UR5MoveIt() # Initialise class object node

    # Obtain prerequisite data
    joint_angles = joint_angles_data()
    env_values = env_data()

    # Initialise subscribers
    subscriber_init()

    # Start execution
    controller()
