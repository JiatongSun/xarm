#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose


class MoveItObstaclesDemo:
    def __init__(self):
        # Initialize move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize ROS node
        rospy.init_node('moveit_obstacles_demo')

        # Initialize scene
        scene = PlanningSceneInterface()

        # Create a publisher for scene change information
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)

        # Create a dictionary to store object color
        self.colors = dict()

        # Wait for scene to be ready
        rospy.sleep(1)

        # Initialize arm_group which needs to be controlled by move_group
        arm = MoveGroupCommander('arm')

        # Get end link name
        end_effector_link = arm.get_end_effector_link()

        # Set tolerance for position (m) and orientation (rad)
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)

        # Allow replanning if failed
        arm.allow_replanning(True)

        # Set reference frame for target position
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        # Set planning time limitation
        arm.set_planning_time(5)

        # Set scene object name
        front_board_id = 'front_board'
        back_board_id = 'back_board'
        left_board_id = 'left_board'
        right_board_id = "right_board"
        bottle_id = 'bottle'

        # Remove previous object
        scene.remove_world_object(front_board_id)
        scene.remove_world_object(back_board_id)
        scene.remove_world_object(left_board_id)
        scene.remove_world_object(right_board_id)
        scene.remove_world_object(bottle_id)
        rospy.sleep(1)

        # Move arm to initial position
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)

        # Initialize gripper_group which needs to be controlled by move_group
        gripper = moveit_commander.MoveGroupCommander('gripper')

        # Set object dimension
        front_board_size = [0.2, 0.25, 0.01]
        back_board_size = [0.2, 0.25, 0.01]
        left_board_size = [0.25, 0.14, 0.01]
        right_board_size = [0.25, 0.14, 0.01]
        bottle_height, bottle_radius = 0.06, 0.02

        # Add object to scene
        front_board_pose = PoseStamped()
        front_board_pose.header.frame_id = reference_frame
        front_board_pose.pose.position.x = 0
        front_board_pose.pose.position.y = 0.1
        front_board_pose.pose.position.z = 0.125
        front_board_pose.pose.orientation.w = 0.707
        front_board_pose.pose.orientation.x = 0.707
        scene.add_box(front_board_id, front_board_pose, front_board_size)

        back_board_pose = PoseStamped()
        back_board_pose.header.frame_id = reference_frame
        back_board_pose.pose.position.x = 0
        back_board_pose.pose.position.y = 0.24
        back_board_pose.pose.position.z = 0.125
        back_board_pose.pose.orientation.w = 0.707
        back_board_pose.pose.orientation.x = 0.707
        scene.add_box(back_board_id, back_board_pose, back_board_size)

        left_board_pose = PoseStamped()
        left_board_pose.header.frame_id = reference_frame
        left_board_pose.pose.position.x = 0.1
        left_board_pose.pose.position.y = 0.17
        left_board_pose.pose.position.z = 0.125
        left_board_pose.pose.orientation.w = 0.707
        left_board_pose.pose.orientation.y = 0.707
        scene.add_box(left_board_id, left_board_pose, left_board_size)

        right_board_pose = PoseStamped()
        right_board_pose.header.frame_id = reference_frame
        right_board_pose.pose.position.x = -0.1
        right_board_pose.pose.position.y = 0.17
        right_board_pose.pose.position.z = 0.125
        right_board_pose.pose.orientation.w = 0.707
        right_board_pose.pose.orientation.y = 0.707
        scene.add_box(right_board_id, right_board_pose, right_board_size)

        bottle_pose = PoseStamped()
        bottle_pose.header.frame_id = reference_frame
        bottle_pose.pose.position.x = 0.1
        bottle_pose.pose.position.y = -0.2
        bottle_pose.pose.position.z = 0.02
        bottle_pose.pose.orientation.w = 0.707
        bottle_pose.pose.orientation.y = 0.707
        scene.add_cylinder(bottle_id, bottle_pose, bottle_height, bottle_radius)

        # Set object color
        self.setColor(front_board_id, 1.0, 1.0, 1.0, 0.6)
        self.setColor(back_board_id, 1.0, 1.0, 1.0, 0.6)
        self.setColor(left_board_id, 1.0, 1.0, 1.0, 0.6)
        self.setColor(right_board_id, 1.0, 1.0, 1.0, 0.6)
        self.setColor(bottle_id, 0.6, 0.1, 0, 1.0)

        # Publish color
        self.sendColors()
        rospy.sleep(3)

        # Set target position for gripper and move gripper
        gripper_position = (np.array([36, -36, 36, 36]) * np.pi / 180.0).tolist()
        gripper.set_joint_value_target(gripper_position)
        gripper.go()
        rospy.sleep(5)

        # Set target position for arm
        joint_position = (np.array([28, -10, 50, -88, 58]) * np.pi / 180.0).tolist()
        arm.set_joint_value_target(joint_position)
        arm.go()

        # Set target position for arm
        joint_position = (np.array([28, -44, 30, -91, 58]) * np.pi / 180.0).tolist()
        arm.set_joint_value_target(joint_position)
        arm.go()

        # Set target position for gripper and move gripper
        gripper_position = (np.array([20, -20, 20, 20]) * np.pi / 180.0).tolist()
        gripper.set_joint_value_target(gripper_position)
        gripper.go()
        rospy.sleep(10)

        # Move arm to initial position
        arm.set_named_target('home')
        arm.go()

        # Set target position for gripper and move gripper
        gripper_position = (np.array([-33, 33, -33, -33]) * np.pi / 180.0).tolist()
        gripper.set_joint_value_target(gripper_position)
        gripper.go()

        # Close and exit MoveIt
        moveit_commander.roscpp_shutdown()

    # Set object color
    def setColor(self, name, r, g, b, a=0.9):
        # Initialize MoveIt color object
        color = ObjectColor()

        # Set color value
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # Update dictionary
        self.colors[name] = color

    # Send color to MoveIt scene
    def sendColors(self):
        # Initialize scene object
        p = PlanningScene()

        # Declare if scene is different
        p.is_diff = True

        # Obtain color value from dictionary
        for color in self.colors.values():
            p.object_colors.append(color)

        # Publish scene object color setting
        self.scene_pub.publish(p)


if __name__ == "__main__":
    try:
        MoveItObstaclesDemo()
    except KeyboardInterrupt:
        raise
