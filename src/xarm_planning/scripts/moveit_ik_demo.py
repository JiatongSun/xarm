#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MoveItIkDemo:
    def __init__(self):
        # Initialize move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize ROS node
        rospy.init_node('moveit_ik_demo')

        # Initialize arm_group which needs to be controlled by move_group
        arm = moveit_commander.MoveGroupCommander('arm')

        # Get end link name
        end_effector_link = arm.get_end_effector_link()

        # Set reference frame for target position
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        # Allow replanning if failed
        arm.allow_replanning(True)
        # print(arm.get_current_pose())
        # print(arm.get_current_rpy())

        # Set tolerance for position (m) and orientation (rad)
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)

        # Move arm to initial position
        arm.set_named_target('home')
        arm.go()

        # Use forward position as target
        forward_pose = arm.get_current_pose().pose

        # Set target position and quaternion for arm workspace
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = forward_pose.position.x
        target_pose.pose.position.y = forward_pose.position.y
        target_pose.pose.position.z = forward_pose.position.z
        target_pose.pose.orientation.x = forward_pose.orientation.x
        target_pose.pose.orientation.y = forward_pose.orientation.y
        target_pose.pose.orientation.z = forward_pose.orientation.z
        target_pose.pose.orientation.w = forward_pose.orientation.w

        # Set current state as initial state
        arm.set_start_state_to_current_state()

        # Set target position and pose of arm
        arm.set_pose_target(target_pose, end_effector_link)

        # Plan trajectory
        traj = arm.plan()

        # Move arm along the planned trajectory
        arm.execute(traj)

        # Move end effector 5cm to the right
        arm.shift_pose_target(2, -0.01, end_effector_link)
        arm.go()

        # Rotate end effector 90 degrees counterclockwise
        arm.shift_pose_target(5, -np.pi / 2, end_effector_link)
        arm.go()

        # Move arm to initial position
        arm.set_named_target('home')
        arm.go()

        # close and exit moveit
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        MoveItIkDemo()
    except rospy.ROSException:
        pass
