#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32

class DistancePublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('distance_publisher_node')
        
        # ROS publisher
        self.distance_pub = rospy.Publisher('perception/distance', Int32, queue_size=10)
        
        # Initialize serial port (Replace '/dev/ttyUSB0' with the correct port)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        
    def read_and_publish_distance(self):
        rate = rospy.Rate(10)  # 10 Hz
        try:
            while not rospy.is_shutdown():
                if self.ser.in_waiting > 0:
                    # Read the distance data from the serial port
                    distance = self.ser.readline().decode('utf-8').strip()
                    if distance:
                        try:
                            # Convert to integer and publish
                            distance_int = int(distance)
                            if distance_int > 0:
                                self.distance_pub.publish(distance_int)
                                logerr.info(f"Received distance : {distance}")
                        except ValueError:
                            rospy.logwarn(f"Received corrupted data : {distance}")
                rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS node interrupted")
        finally:
            self.ser.close()

if __name__ == '__main__':
    distance_publisher = DistancePublisher()
    distance_publisher.read_and_publish_distance()
