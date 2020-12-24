#! usr/bin/env python
'''
Library of UR5-specific functions for MoveIt! and RViz Motion Planning.

Contains:
1) MoveIt! Parameters and Controllers for controlling the arm
    a) Go to a specified pose
    b) Go to specified join angles
2) RViz Planning Scene controls for adding, attaching, detaching and removing objects from the Planning Scene
'''

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import actionlib
import tf


class UR5MoveIt:
    '''
    Class object for the UR5 arm.
    '''

    def __init__(self):
        '''
        Constructor containing all essential assets for MoveIt! and RViz.
        '''

        # Initialise the Node
        rospy.init_node('node_ur5', anonymous=True)

        # Transform Listener for box TF detection
        self.tf_listener = tf.TransformListener()

        # RViz and MoveIt parameters
        self._box_name = "box"
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._execute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._touch_links = self._robot.get_link_names(
            group=self._planning_group)

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):
        '''
        Goes to a specified pose and orientation.

        Parameters:
            arg_pose (Pose object): The pose and orientation to execute planning towards.

        Returns:
            flag_plan (bool): Confirmation whether the planning and execution was successful or not.
        '''
        # Get current pose values
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        # Set final pose target with given pose value
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        # Get final joints values calculated
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        # Verify planning/executing success
        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_joint_angles(self, arg_list_joint_angles):
        '''
        Goes to specified joint angles.

        Parameters:
            arg_list_joint_angles (float[]): A list of joint angles in radians to plan towards.

        Returns:
            flag_plan (bool): Confirmation whether the planning and execution was successful or not.
        '''

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def add_box(self, box_name, box_length, box_pose):
        '''
        Adds a box to the RViz planning scene.

        Parameters:
            box_name (str): The name to be assigned to the box.
            box_length (float): The size of the box.
            box_pose (PoseStamped object): The pose and orientation of the box.
        '''
        self._scene.add_box(box_name,
                            box_pose,
                            size=(box_length, box_length, box_length))

    def attach_box(self, box_name):
        '''
        Attaches the specified object(box) to the robot hand.

        Parameters:
            box_name (str): The name of the box in the RViz Planning Scene te be attached.
        '''
        self._scene.attach_box(self._eef_link,  # The end-effector link
                               box_name,
                               touch_links=self._touch_links)

    def detach_box(self, box_name):
        '''
        Detaches the specified object(box) from the robot hand.

        Parameters:
            box_name (str): The name of the box in the RViz Planning Scene te be detached.
        '''
        self._scene.remove_attached_object(self._eef_link,  # The end-effector link
                                           name=box_name)

    def remove_box(self, box_name):
        '''
        Removes the specified object(box) from the RViz Planning Scene.

        Parameters:
            box_name (str): The name of the box to be removed.
        '''
        self._scene.remove_world_object(box_name)

    def __del__(self):
        '''
        Destructor for the class object.
        '''
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
