#!/usr/bin/env python3
import sys
import rospy
import requests
from std_msgs.msg import Int32
from loguru import logger

class URControllerNode:
    def __init__(self, api_endpoint):
        rospy.init_node('ur_controller_node')
        logger.remove(0)
        logger.add(sys.stderr, format="<red>[{level}]</red> <green>{message}</green> ", colorize=True)
        self.petak_sub = rospy.Subscriber("perception/hand_petak", Int32, self.petak_callback)
        self.api_endpoint = api_endpoint
        self.current_petak = None

    def petak_callback(self, msg):
        petak = msg.data
        if self.current_petak != petak:
            if self.current_petak is not None:
                self.set_relay_state(self.current_petak, 0)  # Send state 0 to the last petak id

            self.current_petak = petak
            self.set_relay_state(petak, 1)

    def set_relay_state(self, petak, state):
        url = f"{self.api_endpoint}/relay/{petak}"
        data = {'state': state}
        try:
            response = requests.post(url, data=data)
            response.raise_for_status()
            logger.info(f"Successfully set relay {petak} to state {state}")
        except requests.exceptions.RequestException as e:
            logger.error(f"Error setting relay state: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        if len(sys.argv) == 2:
            api_endpoint = sys.argv[1]
            node = URControllerNode(api_endpoint)
            node.run()
        else:
            print("Usage: rosrun your_package_name your_script_name.py <api_endpoint>")
    except rospy.ROSInterruptException:
        pass
