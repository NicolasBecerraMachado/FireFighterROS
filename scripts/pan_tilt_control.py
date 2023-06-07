#!/usr/bin/env python
import rospy
import sys
import socket
import numpy as np
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_msgs.msg import String

class PanTiltNode():
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        ####################### SUBSCRIBERS ############################
        rospy.Subscriber("autonomous_control_on", Bool, self.auto_control_cb)

        ####################### PUBLISHERS ############################ 
        self.pub_pan = rospy.Publisher('arduino/pan', UInt8, queue_size=10)
        self.pub_tilt = rospy.Publisher('arduino/tilt', UInt8, queue_size=10)
        self.pub_fire = rospy.Publisher("fire_detection", String, queue_size=10)
        
        ######################## CONSTANTS AND VARIABLES ##############################

        ### Socket connection to vision script (ROS melodic does not support Python3)
        #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #s.connect((socket.gethostname(),1234))
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((socket.gethostname(),1234))
        s.listen(4)
        
        ## AUTONOMOUS CONTROL
        self.autonomous_control_on = False
        self.idle_servos = True

        ## PAN TILT
        self.pan_msg = UInt8()
        self.tilt_msg = UInt8()
        self.pan_angle = 100
        self.tilt_angle = 70

        ## FIRE DATA
        self.fire_homing_enabled = False
        self.fire_in_water_range = False
        self.fire_angle = np.inf
        self.fire_msg = String()

        ##ROS
        r = rospy.Rate(10) #10Hz is the lidar's frequency 
        print("Node initialized 1hz")

        clientSocket, _ = s.accept()

        ############################### MAIN LOOP #####################################
        while not rospy.is_shutdown():
            
            if self.autonomous_control_on:
                
                decode_vision_message(s.recv(40))
                send_fire_data()

                if self.fire_in_water_range:
                    aim_pan_tilt()
                    #TODO: Add method shooting water 

            elif self.idle_servos:
                reset_servos()
                idle_servos = False
        
        r.sleep()

    ############################### METHODS #####################################
    def decode_vision_message(self, message):
        message = message.decode("utf-8")
        message = message.split(",")

        if(len(message) > 0):
            self.pan_angle = int(message[0])
            self.tilt_angle = int(message[1])
            self.fire_homing_enabled = True if message[2] == "1" else False
            self.fire_in_water_range = True if message[3] == "1" else False
            self.fire_angle = int(message[4])

        else:
            self.pan_angle = 100
            self.tilt_angle = 70
            self.fire_homing_enabled = False
            self.fire_in_water_range = False
            self.fire_angle = np.inf

    
    def aim_pan_tilt(self):
        self.pan_msg.data = self.pan_angle
        self.tilt_msg.data = self.tilt_angle if self.tilt_angle > 20 else 20
        self.pub_pan.publish(pan_msg)
        self.pub_tilt.publish(tilt_msg)
    
    def reset_servos():
        self.pan_msg.data = 100
        self.tilt_msg.data = 70
        self.pub_pan.publish(pan_msg)
        self.pub_tilt.publish(tilt_msg)

    def send_fire_data(self):
        homing_enabled = "1" if self.fire_homing_enabled else "0"
        water_range = "1" if self.fire_in_water_range else "0"
        angle = str(self.fire_angle) if self.fire_angle != np.inf else "inf"

        self.fire_msg.data = homing_enabled + water_range + angle
        self.pub_fire.publish(self.fire_msg)

    ############################### CALLBACKS #####################################
    def auto_control_cb(self, msg):
        if self.autonomous_control_on and not msg.data:
            self.idle_servos = True

        self.autonomous_control_on = msg.data

    ############################### CLEANUP ##################################### 
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.self.direction.linear.x = 0
        cleanup_message = UInt8()
        self.pub_pan.publish(cleanup_message)
        self.pub_tilt.publish(cleanup_message)

        cleanup_message = String()
        self.pub_fire.publish(cleanup_message)
        rospy.logwarn("PanTiltNode is dead")
        print("PanTiltNode is dead")

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("PanTiltNode", anonymous=True) 
    PanTiltNode()

