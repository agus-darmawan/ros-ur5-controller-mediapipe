#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import cv2 as cv
import mediapipe as mp
import numpy as np
import sys

mpHand = mp.solutions.hands
mpDraw = mp.solutions.drawing_utils
hands = mpHand.Hands(min_detection_confidence=0.8)

class HandTrackerNode:
    def __init__(self, video_source):
        rospy.init_node('hand_tracker_node', anonymous=True)
        self.image_pub = rospy.Publisher("hand_tracker_image", Image, queue_size=10)
        self.point_pub = rospy.Publisher("hand_position", Point, queue_size=10)
        self.bridge = CvBridge()
        self.video_source = video_source

    def capture_and_publish(self):
        cap = cv.VideoCapture(self.video_source)
        if not cap.isOpened():
            rospy.logerr(f"Error opening video source {self.video_source}")
            return
        
        ws, hs = 1280, 720
        cap.set(3, ws)
        cap.set(4, hs)

        while not rospy.is_shutdown() and cap.isOpened():
            success, img = cap.read()
            if not success:
                break
            
            img = cv.flip(img, 1)
            img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
            results = hands.process(img_rgb)
            img_rgb = cv.cvtColor(img_rgb, cv.COLOR_RGB2BGR)

            multi_hand_detection = results.multi_hand_landmarks
            lmList = []

            if multi_hand_detection:
                for id, lm in enumerate(multi_hand_detection):
                    mpDraw.draw_landmarks(img_rgb, lm, mpHand.HAND_CONNECTIONS,
                                          mpDraw.DrawingSpec(color=(0, 255, 255), thickness=4, circle_radius=7),
                                          mpDraw.DrawingSpec(color=(0, 0, 0), thickness=4))
                
                single_hand_detection = multi_hand_detection[0]
                for lm in single_hand_detection.landmark:
                    h, w, c = img_rgb.shape
                    lm_x, lm_y = int(lm.x * w), int(lm.y * h)
                    lmList.append([lm_x, lm_y])
                
                if lmList:
                    px, py = lmList[8]
                    cv.circle(img_rgb, (px, py), 15, (255, 0, 255), cv.FILLED)
                    cv.putText(img_rgb, str((px, py)), (px + 10, py - 10), cv.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 3)


                    print(f'Hand Position x: {px} y: {py}')
                    # Publish hand position as Point message
                    hand_point_msg = Point()
                    hand_point_msg.x = px
                    hand_point_msg.y = py
                    hand_point_msg.z = 0
                    self.point_pub.publish(hand_point_msg)

            # Publish image as sensor_msgs/Image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_rgb, "bgr8"))
            except CvBridgeError as e:
                print(e)

            cv.imshow("Image", img_rgb)
            key = cv.waitKey(1)
            if key == ord('q'):
                break
        
        cap.release()
        cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        if len(sys.argv) == 2:
            video_source = sys.argv[1]
            node = HandTrackerNode(video_source)
            node.capture_and_publish()
        else:
            print("Usage: rosrun your_package_name your_script_name.py <video_file_path or camera_index>")
    except rospy.ROSInterruptException:
        pass
