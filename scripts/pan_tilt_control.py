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
        self.pub_pump = rospy.Publisher("arduino/pump", UInt8, queue_size=10)
        self.pub_turn = rospy.Publisher("turn_robot", Bool, queue_size=10)
        
        ######################## CONSTANTS AND VARIABLES ##############################

        ### Socket connection to vision script (ROS melodic does not support Python3)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((socket.gethostname(),1234))
        s.listen(4)
        
        ## AUTONOMOUS CONTROL
        self.autonomous_control_on = False

        ## PAN TILT
        self.pan_msg = UInt8()
        self.tilt_msg = UInt8()
        self.pan_angle = 100
        self.tilt_angle = 70
        self.reset_servos()

        ## FIRE DATA
        self.fire_detected = False
        self.fire_in_water_range = False
        self.fire_angle = np.inf
        self.fire_msg = String()

        ## WATER SPRAY CONTROL
        self.water_spray_enabled = False
        self.spray_base_pan = self.pan_angle
        self.spray_base_tilt = self.tilt_angle
        self.spray_step = 0.5
        self.spray_step_counter = 0
        self.spray_direction = 1
        self.spray_cycle_counter = 0
        self.pump_msg = UInt8()

        ##AFTER SPRAY WAIT
        self.wait_after_spray_enabled = False
        self.after_spray_wait_counter = 0
        
        ##AFTER FIRE EXTINGUISHED TURN
        self.turn_robot_enabled = False
        self.turn_msg = Bool()
        self.turn_msg.data = False

        ##ROS
        r = rospy.Rate(30)
        print("Node initialized 30hz")
        print("Waiting for vision_jetson.py")
        self.clientSocket, _ = s.accept()
        print("Listening to vision_jetson.py")

        ############################### MAIN LOOP #####################################
        while not rospy.is_shutdown():
            
            vision_message = self.clientSocket.recv(40)
            if self.autonomous_control_on:
                
                #if self.water_spray_enabled:
                #    self.spray_water()

                #elif self.wait_after_spray_enabled:
                #    self.wait_after_spray(vision_message)

                #elif self.turn_robot_enabled:
                #    self.turn_robot()

                #else:
                self.search_fire(vision_message)
            print(vision_message)
            r.sleep()

    ############################### METHODS #####################################
    def turn_robot(self):
	print("turn_robot")
        self.turn_robot_enabled = False
        self.turn_msg.data = True
        self.pub_turn.publish(self.turn_msg)
        self.turn_msg.data = False

    def wait_after_spray(self, vision_message):
	print("wait_after_spray")
        if self.after_spray_wait_counter >= 30:
            self.wait_after_spray_enabled = False
            self.after_spray_wait_counter = 0
            self.decode_vision_message(vision_message)

            if self.fire_detected:
                self.water_spray_enabled = False #Change this later
            
            else:
                self.turn_robot_enabled = True
                self.water_spray_enabled = False

        else:
            self.after_spray_wait_counter += 1


    def search_fire(self, vision_message):
	print("search_fire")        
	self.decode_vision_message(vision_message)
        self.send_fire_data()

        if self.fire_in_water_range:
            self.aim_pan_tilt()
        elif not self.fire_detected:
            self.reset_servos()


    def spray_water(self):
        print("spray_water")
        self.spray_step_counter = self.spray_step_counter + self.spray_direction

        current_pan = self.spray_base_pan + (self.spray_step * self.spray_step_counter)
        current_tilt = self.spray_base_tilt + (self.spray_step * self.spray_step_counter)

        if self.spray_step_counter >= 5:
            self.spray_direction = -1
            self.spray_cycle_counter += 1
        elif self.spray_step_counter <= -5:
            self.spray_direction = 1
            self.spray_cycle_counter += 1
        
        if self.spray_cycle_counter >= 5:
            self.pump_msg.data = 0	
	    self.spray_step_counter = 0
	    self.spray_cycle_counter = 0
            self.water_spray_enabled = False
            self.wait_after_spray_enabled = True
        else:
            self.pump_msg.data = 100
        
        self.pan_msg.data = abs(int(current_pan))
        self.tilt_msg.data = abs(int(current_tilt)) if current_tilt > 20 else 20
        #print("Spray tilt: " + str(self.tilt_msg.data))
        #print("Spray pan: " + str(self.pan_msg.data))        
	self.pub_pan.publish(self.pan_msg)
        self.pub_tilt.publish(self.tilt_msg)
        self.pub_pump.publish(self.pump_msg)


    def decode_vision_message(self, message):
        message = message.decode("utf-8")
        
        message = message.split(",")
        prev_spray_enabled = self.water_spray_enabled	
        
        if(len(message) > 0):
            self.pan_angle = int(message[0]) if message[0].isnumeric() else 0
            self.tilt_angle = int(message[1]) if message[1].isnumeric() else 0
            self.fire_detected = True if message[2] == "1" else False
            self.fire_in_water_range = True if message[3] == "1" else False
            self.fire_angle = int(message[4]) if message[4] != "inf" else np.inf
            self.water_spray_enabled = True if message[5] == "1" else False
            print("tilt: " + str(self.tilt_angle))
            print("pan: " + str(self.pan_angle))

        else:
            self.pan_angle = 100
            self.tilt_angle = 70
            self.fire_detected = False
            self.fire_in_water_range = False
            self.fire_angle = np.inf
            self.water_spray_enabled = False
        
        if not prev_spray_enabled and self.water_spray_enabled:
            self.spray_base_tilt = self.tilt_angle
            self.spray_base_pan = self.pan_angle

    def aim_pan_tilt(self):
        self.pan_msg.data = self.pan_angle
        self.tilt_msg.data = self.tilt_angle if self.tilt_angle > 20 else 20
        self.pub_pan.publish(self.pan_msg)
        self.pub_tilt.publish(self.tilt_msg)
    
    def reset_servos(self):
        self.pan_msg.data = 100
        self.tilt_msg.data = 70
        self.pub_pan.publish(self.pan_msg)
        self.pub_tilt.publish(self.tilt_msg)

    def send_fire_data(self):
        homing_enabled = "1" if self.fire_detected else "0"
        water_range = "1" if self.fire_in_water_range else "0"
        angle = self.fire_angle if self.fire_angle != np.inf else "inf"

        self.fire_msg.data = "{},{},{}".format(homing_enabled,water_range,angle)
        self.pub_fire.publish(self.fire_msg)

    ############################### CALLBACKS #####################################
    def auto_control_cb(self, msg):
        if self.autonomous_control_on and not msg.data:
            self.reset_servos()

        self.autonomous_control_on = msg.data

    ############################### CLEANUP ##################################### 
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.self.direction.linear.x = 0
        cleanup_message = UInt8()
        cleanup_message.data = 100
        self.pub_pan.publish(cleanup_message)
        cleanup_message.data = 70
        self.pub_tilt.publish(cleanup_message)
        cleanup_message.data = 0
        self.pub_pump.publish(cleanup_message)

        cleanup_message = String()
        self.pub_fire.publish(cleanup_message)
        rospy.logwarn("PanTiltNode is dead")
        print("PanTiltNode is dead")
        self.clientSocket.close()

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("PanTiltNode", anonymous=True) 
    PanTiltNode()

