#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

        # Set tolerance for position (m) and orientation (rad)
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)

        # Move arm to initial position
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)

        # Set target position and quaternion for arm workspace
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = 0.1
        target_pose.pose.position.y = 0.1
        target_pose.pose.position.z = 0.1
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 1.0
        target_pose.pose.orientation.w = 0.0

        # Set current state as initial state
        arm.set_start_state_to_current_state()

        # Set target position and pose of arm
        arm.set_pose_target(target_pose, end_effector_link)

        # Plan trajectory
        traj = arm.plan()

        # Move arm along the planned trajectory
        arm.execute(traj)
        rospy.sleep(1)



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
