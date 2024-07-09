#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Int32
import actionlib
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from loguru import logger
import math

class URInterfaceNode:
    def __init__(self):
        rospy.init_node('ur_interface_node')
        logger.remove(0)
        logger.add(sys.stderr, format="<red>[{level}]</red> <green>{message}</green> ", colorize=True)
        self.petak_sub = rospy.Subscriber("controller/command", Int32, self.command_callback)
        self.trajectory_client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.trajectory_client.wait_for_server()

    def move_to_joint_positions(self, positions):
        goal = FollowJointTrajectoryActionGoal()
        goal.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(1)
        goal.goal.trajectory.points.append(point)
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()
        self.result = self.trajectory_client.get_result()

        if self.result.error_code == 0:
            logger.info("Successfully moved to joint positions")
            return True
        else:
            logger.error("Failed to move to joint positions")
            return False

    def degrees_to_radians(self, degrees_list):
        return [math.radians(deg) for deg in degrees_list]

    def command_callback(self, msg):
        command = msg.data
        logger.info(f"Received command: {command}")

        if command == 1:
            # Move to positions for command 1
            positions = self.degrees_to_radians([76.20, -53.94, 66.04, -101.07, -88.16, -10.69])
            self.move_to_joint_positions(positions)
            positions = self.degrees_to_radians([81.28, -40.58, 52.64, -101.86, -91.09, -10.63])
            self.move_to_joint_positions(positions)
            # Open gripper (implement this functionality)

            # Send topic int wait for put location 0
            self.send_put_location(0)

            # Move to idle position
            idle_positions = self.degrees_to_radians([38.50, -88.87, 90.25, -89.22, -87.04, -11.36])
            self.move_to_joint_positions(idle_positions)

        elif command == 2:
            # Move to positions for command 2
            positions = self.degrees_to_radians([87.69, -60.15, 73.69, -102.77, -87.05, -10.63])
            self.move_to_joint_positions(positions)
            positions = self.degrees_to_radians([88.30, -54.13, 76.61, -107.11, -88.09, -10.63])
            self.move_to_joint_positions(positions)
            # Open gripper (implement this functionality)

            # Send topic int wait for put location 0
            self.send_put_location(0)

            # Move to idle position
            idle_positions = self.degrees_to_radians([38.50, -88.87, 90.25, -89.22, -87.04, -11.36])
            self.move_to_joint_positions(idle_positions)

        elif command == 3:
            # Move to positions for command 3
            positions_list = [
                [7.92, -90.36, 90.43, -90.51, -88.37, -31.97],
                [7.89, -86.83, 106.39, -111.66, -89.41, -31.99],
                [9.50, -80.34, 116.48, -125.18, -92.80, -31.99]
            ]
            for positions in positions_list:
                positions_rad = self.degrees_to_radians(positions)
                self.move_to_joint_positions(positions_rad)
            
            # Close gripper (implement this functionality)

            # Move to next position
            next_positions = self.degrees_to_radians([78.58, -84.25, 83.42, -93.19, -87.07, -10.64])
            self.move_to_joint_positions(next_positions)

            # Send topic int wait for put location 1
            self.send_put_location(1)

        elif command == 4:
            # Move to positions for command 4
            positions_list = [
                [39.24, -61.52, 73.86, -97.26, -87.08, -10.64],
                [39.25, -58.28, 83.66, -110.15, -87.07, -10.64]
            ]
            for positions in positions_list:
                positions_rad = self.degrees_to_radians(positions)
                self.move_to_joint_positions(positions_rad)
            
            # Close gripper (implement this functionality)

            # Move to next position
            next_positions = self.degrees_to_radians([78.58, -84.25, 83.42, -93.19, -87.07, -10.64])
            self.move_to_joint_positions(next_positions)

            # Send topic int wait for put location 1
            self.send_put_location(1)

    def send_put_location(self, location):
        # Implement sending the integer topic and waiting functionality
        rospy.loginfo(f"Sending put location {location}")
        # Add your code here to publish to a topic or perform the necessary action

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    
    try:
        node = URInterfaceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
