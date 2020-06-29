#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import sys
import moveit_commander
from control_msgs.msg import GripperCommand


class MoveItFkDemo:
    def __init__(self):
        # Initialize move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize ROS node
        rospy.init_node('moveit_fk_demo', anonymous=True)

        # Initialize arm_group which needs to be controlled by move_group
        arm = moveit_commander.MoveGroupCommander('arm')

        # Initialize gripper_group which needs to be controlled by move_group
        gripper = moveit_commander.MoveGroupCommander('gripper')

        # Set tolerance of error for arm and gripper
        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)

        # Move arm to initial position
        arm.set_named_target('home')
        arm.go()

        # Set target position for gripper and move gripper
        gripper_position = (np.array([-20, 20, -20, -20]) * np.pi / 180.0).tolist()
        gripper.set_joint_value_target(gripper_position)
        gripper.go()

        # Set target position for arm
        joint_position = (np.array([0, -58, 89, 52, 0]) * np.pi / 180.0).tolist()
        arm.set_joint_value_target(joint_position)

        # Control arm to move
        arm.go()

        # Close and exit MoveIt
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        MoveItFkDemo()
    except rospy.ROSException:
        pass

