#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

        # set tolerance of error for arm and gripper
        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)

        # move arm to initial position
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)

        # set target position for gripper and move gripper
        gripper.set_joint_value_target([0.01])
        gripper.go()
        rospy.sleep(1)

        # set target position for arm
        joint_position = [-0.05, -0.05, -0.05, -0.05, -0.05, -0.05]
        arm.set_joint_value_target(joint_position)

        # control arm to move
        arm.go()
        rospy.sleep(1)

        # close and exit moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == '__main__':
    try:
        MoveItFkDemo()
    except rospy.ROSException:
        pass

