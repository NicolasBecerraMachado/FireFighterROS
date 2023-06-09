#!/usr/bin/env python 
import rospy
import math
import numpy as np
import tty
import sys
import termios
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool

class MasterNode(): 

    ######### Constructor ##########
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        ####################### PUBLISHERS ############################ 
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.auto_control_pub = rospy.Publisher("autonomous_control_on", Bool, queue_size=1)
        
        
        ######################## CONSTANTS AND VARIABLES ############################## 
        
        ##Autonomous control
        self.autonomous_control_on = False
        self.stop_robot = False

        ##ROS
        self.vel_msg = Twist()
        self.auto_control_msg = Bool()
        self.auto_control_msg.data = False
        r = rospy.Rate(10) #10Hz 
        print("Node initialized 10hz")
        self.cmd_vel_pub.publish(self.vel_msg)
        self.auto_control_pub.publish(self.auto_control_msg)

        ##Put terminal in raw mode (No enter required)
        self.orig_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

        ############################### MAIN LOOP #####################################
        while not rospy.is_shutdown():

            key = sys.stdin.read(1)[0]
            self.process_input(key.lower())

            r.sleep()

    ############################### METHODS #####################################
    def process_input(self, key):
        if(key == 'w'):
            self.publish_vel_msg(0.3, 0.0, 0.0)

        elif(key == 'a'):
            self.publish_vel_msg(0.0, 0.3, 0.0)

        elif(key == 's'):
            self.publish_vel_msg(-0.3, 0.0, 0.0)

        elif(key == 'd'):
            self.publish_vel_msg(0.0, -0.3, 0.0)

        elif(key == 'q'):
            self.publish_vel_msg(0.0, 0.0, 0.8)

        elif(key == 'e'):
            self.publish_vel_msg(0.0, 0.0, -0.8)

        elif(key == 'o'):
            self.toggle_and_publish_auto_control_msg()
        
        elif(key == chr(32)):
            self.publish_vel_msg(0.0, 0.0, 0)

        else:
            pass
            

    def publish_vel_msg(self, linear_x, linear_y, angular_z):

        self.vel_msg.linear.x = linear_x
        self.vel_msg.linear.y = linear_y
        self.vel_msg.angular.z = angular_z

        self.cmd_vel_pub.publish(self.vel_msg)
            
    def toggle_and_publish_auto_control_msg(self):
        self.autonomous_control_on = True if not self.autonomous_control_on else False
        self.auto_control_msg.data = self.autonomous_control_on
        self.auto_control_pub.publish(self.auto_control_msg)

    ############################### CLEANUP #####################################    
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        cleanup_message = Twist()
        self.cmd_vel_pub.publish(cleanup_message)
        cleanup_message = Bool()
        self.auto_control_pub.publish(cleanup_message)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings) 

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("master_node", anonymous=True) 
    MasterNode()
