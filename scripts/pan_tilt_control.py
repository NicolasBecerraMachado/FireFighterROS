#!/usr/bin/env python
import rospy
import sys
import socket
from std_msgs.msg import UInt8

class PanTiltNode():
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        #define ros publishers
        pubPan = rospy.Publisher('arduino/pan',UInt8,queue_size=10)
        pubTilt = rospy.Publisher('arduino/tilt',UInt8,queue_size=10)

        message = UInt8()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((socket.gethostname(),1234))
	
        panAngle = 130
        tiltAngle = 90
        while not rospy.is_shutdown():
            angles = s.recv(15).decode("utf-8")
            print(angles)
            angles_list = angles.split(",")
            print(angles_list)
            if(len(angles) > 0):
                panAngle = int(angles_list[0])
                tiltAngle = int(angles_list[1])

	    message.data = panAngle
	    print(message.data)
	    pubPan.publish(message)
	    message.data = tiltAngle
	    print(message.data)
	    pubTilt.publish(message)

    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.self.direction.linear.x = 0
        #message = UInt8()
        #message.data = int(130)
	#self.pubPan.publish(message)
	#message.data = int(90)
	#self.pubTilt.publish(message)
        rospy.logwarn("PanTiltNode is dead")
        print("PanTiltNode is dead")

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("PanTiltNode", anonymous=True) 
    PanTiltNode()

