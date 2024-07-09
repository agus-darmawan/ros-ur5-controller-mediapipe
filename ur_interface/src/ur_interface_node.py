#!/usr/bin/env python3
import sys
import rospy
import actionlib
import math
from std_msgs.msg import Int32
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from loguru import logger

class URInterfaceNode:
    def __init__(self):
        rospy.init_node('ur_interface_node')
        logger.remove(0)
        logger.add(sys.stderr, format="<red>[{level}]</red> <green>{message}</green>", colorize=True)
        self.cammera_pub = rospy.Publisher("controller/mode/cam", Int32, queue_size=10)
        self.petak_sub = rospy.Subscriber("controller/command", Int32, self.command_callback)
        self.put_location_pub = rospy.Publisher("put_location", Int32, queue_size=10)
        self.trajectory_client = actionlib.SimpleActionClient(
            'scaled_pos_joint_traj_controller/follow_joint_trajectory', 
            FollowJointTrajectoryAction
        )
        self.trajectory_client.wait_for_server()

    def move_to_joint_positions(self, positions):
        goal = FollowJointTrajectoryActionGoal()
        goal.goal.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
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

    @staticmethod
    def degrees_to_radians(degrees_list):
        return [math.radians(deg) for deg in degrees_list]

    def command_callback(self, msg):
        command = msg.data
        logger.info(f"Received command: {command}")

        if command == 1:
            self.execute_command_1()
        elif command == 2:
            self.execute_command_2()
        elif command == 3:
            self.execute_command_3()
        else:
            logger.warning(f"Unknown command received: {command}")

    def execute_command_1(self):
        positions_list = [
            [76.20, -53.94, 66.04, -101.07, -88.16, -10.69],
            [81.28, -40.58, 52.64, -101.86, -91.09, -10.63]
        ]
        
        for positions in positions_list:
            self.move_to_joint_positions(self.degrees_to_radians(positions))
        
        self.open_gripper()
        
        
        idle_positions = [38.50, -88.87, 90.25, -89.22, -87.04, -11.36]
        self.move_to_joint_positions(self.degrees_to_radians(idle_positions))

    def execute_command_2(self):
        positions_list = [
            [87.69, -60.15, 73.69, -102.77, -87.05, -10.63],
            [88.30, -54.13, 76.61, -107.11, -88.09, -10.63]
        ]
        
        for positions in positions_list:
            self.move_to_joint_positions(self.degrees_to_radians(positions))
        
        self.open_gripper()
        
        
        idle_positions = [38.50, -88.87, 90.25, -89.22, -87.04, -11.36]
        self.move_to_joint_positions(self.degrees_to_radians(idle_positions))

    def execute_command_3(self):
        positions_list = [
            [39.24, -61.52, 73.86, -97.26, -87.08, -10.64],
            [39.25, -58.28, 83.66, -110.15, -87.07, -10.64]
        ]
        
        for positions in positions_list:
            self.move_to_joint_positions(self.degrees_to_radians(positions))

        
        self.close_gripper()
       
        next_positions = [78.58, -84.25, 83.42, -93.19, -87.07, -10.64]
        self.move_to_joint_positions(self.degrees_to_radians(next_positions))
        
        mode = Int32()
        mode.data = 2
        self.cammera_pub.publish(mode)

    def open_gripper(self):
        # Implement the functionality to open the gripper
        logger.info("Opening gripper")
        # Add your code here

    def close_gripper(self):
        # Implement the functionality to close the gripper
        logger.info("Closing gripper")
        # Add your code here

    def send_put_location(self, location):
        # Implement sending the integer topic and waiting functionality
        logger.info(f"Sending put location {location}")
        self.put_location_pub.publish(location)
        # Add additional logic if needed

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = URInterfaceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
