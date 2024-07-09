#!/usr/bin/env python3

import sys
import cv2 as cv
import mediapipe as mp
import rospy

from loguru import logger

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32

mpHand = mp.solutions.hands
mpDraw = mp.solutions.drawing_utils
hands = mpHand.Hands(min_detection_confidence=0.9)


class HandTrackerNode:
    def __init__(self, video_source):
        rospy.init_node('hand_tracker_node')
        logger.remove(0)
        logger.add(sys.stderr, format="<red>[{level}]</red> <green>{message}</green> ", colorize=True)
        self.image_pub = rospy.Publisher("perception/hand_tracker_image", Image, queue_size=10)
        self.petak_pub = rospy.Publisher("perception/hand/position", Int32, queue_size=10)

        self.bridge = CvBridge()
        self.video_source = video_source
        self.mode = 0  # Default mode
        self.grid_rows = 4
        self.grid_cols = 2

        rospy.Subscriber("controller/mode/cam", Int32, self.mode_callback)

    def mode_callback(self, data):
        self.mode = data.data
        if self.mode == 1:
            self.grid_rows = 4
            self.grid_cols = 2
        elif self.mode == 2:
            self.grid_rows = 1
            self.grid_cols = 2
        else:
            self.grid_rows = 0
            self.grid_cols = 0

    def capture_and_publish(self):
        cap = cv.VideoCapture(self.video_source)
        if not cap.isOpened():
            rospy.logerr(f"Error opening video source {self.video_source}")
            return
        
        ws, hs = 848, 480
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

            if self.grid_rows > 0 and self.grid_cols > 0:
                section_width = ws // self.grid_cols
                section_height = hs // self.grid_rows

                # Draw grid lines and add text to each petak
                for i in range(1, self.grid_cols):
                    x = i * section_width
                    cv.line(img_rgb, (x, 0), (x, hs), (0, 0, 255), 2)
                
                for i in range(1, self.grid_rows):
                    y = i * section_height
                    cv.line(img_rgb, (0, y), (ws, y), (0, 0, 255), 2)
                
                for row in range(self.grid_rows):
                    for col in range(self.grid_cols):
                        center_x, center_y = self.calculate_center(col, row, section_width, section_height)
                        petak = row * self.grid_cols + col
                        text = f"Petak: {petak}"
                        cv.putText(img_rgb, text, (center_x - 30, center_y + 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv.LINE_AA)

            multi_hand_detection = results.multi_hand_landmarks
            lmList = []

            if multi_hand_detection:
                for id, lm in enumerate(multi_hand_detection):
                    mpDraw.draw_landmarks(img_rgb, lm, mpHand.HAND_CONNECTIONS,
                                          mpDraw.DrawingSpec(color=(0, 255, 255), thickness=2, circle_radius=3),
                                          mpDraw.DrawingSpec(color=(0, 0, 0), thickness=2))
                
                single_hand_detection = multi_hand_detection[0]
                if single_hand_detection:
                    lm = single_hand_detection.landmark[8]
                    h, w, c = img_rgb.shape
                    lm_x, lm_y = int(lm.x * w), int(lm.y * h)
                    
                    if self.grid_rows > 0 and self.grid_cols > 0:
                        petak = self.calculate_petak(lm_x, lm_y, section_width, section_height)
                        cv.circle(img_rgb, (lm_x, lm_y), 7, (255, 0, 0), cv.FILLED)
                        text = f"Petak: {petak}"
                        cv.putText(img_rgb, text, (10, 20), cv.FONT_HERSHEY_PLAIN, 1, (25, 25, 25), 2, cv.LINE_AA)
                        cv.putText(img_rgb, text, (10, 20), cv.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv.LINE_AA)
                        
                        logger.info(f'Hand is in petak: {petak}')
                        # Publish petak as Int32 message
                        self.petak_pub.publish(petak)

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

    def calculate_petak(self, px, py, section_width, section_height):
        col_index = px // section_width
        row_index = py // section_height
        petak = row_index * self.grid_cols + col_index
        return petak
    
    def calculate_center(self, col, row, section_width, section_height):
        center_x = col * section_width + section_width // 2
        center_y = row * section_height + section_height // 2
        return center_x, center_y

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
