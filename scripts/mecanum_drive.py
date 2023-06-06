#!/usr/bin/env python
import rospy 
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

# receive the desired speed from a twist and convert it to angular speeds to each wheel 
# data extracted from https://www.robotpark.com/academy/all-types-of-robots/wheeled-robots/four-wheeled-robots/#:~:text=This%20kind%20of%20robot%20uses,one%20will%20slip%20(inefficient). 
# equations extracted from https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
class ArduinoNode():
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        ############    SUBSCRIBERS   ####################### 
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb) 
        ############ CONSTANTS AND VARIABLES ################ 
        diametro = 0.06
        lx = 0.15
        ly = 0.13
        r = rospy.Rate(10) #10Hz 
        print("ArduinoNode initialized 10hz")
        rospy.logwarn("ArduinoNode initialized 10hz")
        
        self.twist = Twist()

        self.w0_pub = rospy.Publisher('arduino/w0', Float64, queue_size=1)
        self.w1_pub = rospy.Publisher('arduino/w1', Float64, queue_size=1)
        self.w2_pub = rospy.Publisher('arduino/w2', Float64, queue_size=1)
        self.w3_pub = rospy.Publisher('arduino/w3', Float64, queue_size=1)

        self.speeds = [Float64(),Float64(),Float64(),Float64()]

        while not rospy.is_shutdown():
            if self.twist:
                vx = self.twist.linear.x
                vy = self.twist.linear.y
                
                self.speeds[0] = (vx - vy - (lx + ly)*self.twist.angular.z)/(diametro/2)

                self.speeds[1] = (vx + vy + (lx + ly)*self.twist.angular.z)/(diametro/2)

                self.speeds[2] = (vx + vy - (lx + ly)*self.twist.angular.z)/(diametro/2)

                self.speeds[3] = (vx - vy + (lx + ly)*self.twist.angular.z)/(diametro/2)

                

                self.pub_speeds(self.speeds[0],self.speeds[1],self.speeds[2],self.speeds[3])
                
                r.sleep()  #It is very important that the r.sleep function is called at least once every cycle. 
    
    def cmd_vel_cb(self, twist_msg): 
        ## This function receives the Twist message and copies this message to a member of the class
        self.twist = twist_msg

    def pub_speeds(self, w1,w2,w3,w4):
        self.w0_pub.publish(w1)
        self.w1_pub.publish(w2)
        self.w2_pub.publish(w3)
        self.w3_pub.publish(w4)
    
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.self.direction.linear.x = 0
        self.pub_speeds(0,0,0,0)
        rospy.logwarn("ArduinoNode is dead")
        print("ArduinoNode is dead") 

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("Arduino_Node", anonymous=True) 
    ArduinoNode()
