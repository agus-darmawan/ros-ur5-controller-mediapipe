#!/usr/bin/env python3
import sys
import rospy
import requests
from std_msgs.msg import Int32
from ur_srvs.srv import StartService, StartServiceResponse, StartServiceRequest
from ur_srvs.srv import ModeService, ModeServiceResponse, ModeServiceRequest
from loguru import logger

class URControllerNode:
    def __init__(self):
        rospy.init_node('ur_controller_node')
        logger.remove(0)
        logger.add(sys.stderr, format="<red>[{level}]</red> <green>{message}</green> ", colorize=True)
        rospy.Subscriber("perception/hand/position", Int32, self.petak_callback)
        rospy.Subscriber("perception/distance", Int32, self.distance_callback)

        self.command_pub = rospy.Publisher("controller/command", Int32, queue_size=10)
        self.cammera_pub = rospy.Publisher("controller/mode/cam", Int32, queue_size=10)
        self.current_petak = None
        self.is_started = False
        self.mode = 0
        rospy.Service('/controller/start',StartService,self.startCallback)
        rospy.Service('/controller/mode',ModeService,self.modeCallback)
 
    def modeCallback(self,req : ModeServiceRequest) -> ModeServiceResponse:
        mode = Int32()
        ret = ModeServiceResponse()
        if req.mode == '1':
            self.mode == 1
            mode.data = 1
            ret.succes = True
        elif req.mode == '2':
            self.mode == 2
            mode.data = 0
            ret.succes = True
        else:
            self.mode == 0
            mode.data = 0
            ret.succes = False

        self.cammera_pub.publish(mode)
        return ret
 
    def startCallback(self,req : StartServiceRequest) -> StartServiceResponse:
        ret = StartServiceResponse()
        if req.start == 'start':
            self.is_started = True
            ret.succes = True
        elif req.start == 'stop':
            self.is_started = False
            ret.succes = True
        return ret
    
    def petak_callback(self, msg):
        petak = msg.data
        if self.current_petak != petak:
            if self.current_petak is not None:
                self.set_relay_state(self.current_petak, 0)  # Send state 0 to the last petak id
            self.current_petak = petak
            self.set_relay_state(petak, 1)
    
    def distance_callback(self, msg):
        distance = msg.data
        if distance < 15:
            command = Int32()
            command.data = 3
            self.command_pub.publish(command)


    def set_relay_state(self, petak, state):
        if mode == 1:
            url = f"http://localhost:8000/relay/{petak}"
            data = {'state': state}
            try:
                response = requests.post(url, data=data)
                response.raise_for_status()
                logger.info(f"Successfully set relay {petak} to state {state}")
            except requests.exceptions.RequestException as e:
                logger.error(f"Error setting relay state: {e}")
        elif mode == 2:
            command = Int32()
            command.data = petak
            self.command_pub.publish(command)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = URControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass