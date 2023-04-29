#!/usr/bin/env python 
import rospy 
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import math

# Converts the read Float 32 files into pwm Int8 to output to the serialNode
class Remap():
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        ############    SUBSCRIBERS   ####################### 
        rospy.Subscriber("w0", Float32, self.read_w0_cb)
        rospy.Subscriber("w1", Float32, self.read_w1_cb) 
        rospy.Subscriber("w2", Float32, self.read_w2_cb) 
        rospy.Subscriber("w3", Float32, self.read_w3_cb) 
        ############ CONSTANTS AND VARIABLES ################ 

        r = rospy.Rate(10) #10Hz 
        print("Remap initialized 10hz")
        rospy.logwarn("Remap initialized 10hz")
        

        self.min_pwm = [44, 40, 48, 60]
        self.min_speed = [0, 0, 0, 0] #please update
        self.max_speed = [180.8*0.03, 187.0*0.03, 178.2*0.03, 179.046*0.03]

        self.w0 = Float32()
        self.w1 = Float32()
        self.w2 = Float32()
        self.w3 = Float32()

        self.pwm0 = rospy.Publisher('pwm0', Int32, queue_size=10)
        self.pwm1 = rospy.Publisher('pwm1', Int32, queue_size=10)
        self.pwm2 = rospy.Publisher('pwm2', Int32, queue_size=10)
        self.pwm3 = rospy.Publisher('pwm3', Int32, queue_size=10)

        self.pwm = [Int32()]*4

        while not rospy.is_shutdown():
            if self.w0:
                self.update_pwm0(self.w0.data)
            if self.w1:
                self.update_pwm1(self.w1.data)
            if self.w2:
                self.update_pwm2(self.w2.data)
            if self.w3:
                self.update_pwm3(self.w3.data)
            
            self.pub_pwm(self.pwm[0],self.pwm[1],self.pwm[2],self.pwm[3])

            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle. 
    
    # These functions receive the Float32 message and copies this message to a member of the class
    def read_w0_cb(self, float_msg): 
        self.w0 = float_msg
    def read_w1_cb(self, float_msg): 
        self.w1 = float_msg
    def read_w2_cb(self, float_msg): 
        self.w2 = float_msg
    def read_w3_cb(self, float_msg): 
        self.w3 = float_msg

    #####################################
    #These functions are used to update the pwm values based on the minimum and maximum speed
    def update_pwm0(self,w):
        back = False
        if w < 0:
            back = True
        w = abs(w)
        act_pwm = 0
        """
        if w > self.max_speed[0]:
            act_pwm = 255
            print("wheel 0 cannot go faster")
            rospy.logwarn("wheel 0 cannot go faster")
        elif w < self.min_speed[0]:
            act_pwm = self.min_speed[0]
            print("wheel 0 cannot go this slow")
            rospy.logwarn("wheel 0 cannot this slow")
        else:
            act_pwm = ((255 - self.min_pwm[0])/(self.max_speed[0] - self.min_speed[0])) * w
        if back:
            act_pwm -= act_pwm
        """
        act_pwm = ((255 - self.min_pwm[0])/(self.max_speed[0] - self.min_speed[0])) * w
        if back:
            act_pwm = -act_pwm
        self.pwm[0].data = int(act_pwm)
        print("pwm0 = ",str(self.pwm[0].data))

    def update_pwm1(self,w):
        back = False
        if w < 0:
            back = True
        w = abs(w)
        act_pwm = 0
        """
        if w > self.max_speed[1]:
            act_pwm = 255
            print("wheel 1 cannot go faster")
            rospy.logwarn("wheel 1 cannot go faster")
        elif w < self.min_speed[1]:
            act_pwm = self.min_speed[1]
            print("wheel 1 cannot go this slow")
            rospy.logwarn("wheel 1 cannot this slow")
        else:
            act_pwm = ((255 - self.min_pwm[1])/(self.max_speed[1] - self.min_speed[1])) * w
        if back:
            act_pwm -= act_pwm
        """
        act_pwm = ((255 - self.min_pwm[1])/(self.max_speed[1] - self.min_speed[1])) * w
        if back:
            act_pwm = -act_pwm
        self.pwm[1].data = int(act_pwm)
        print("pwm1 = ",str(self.pwm[1].data))

    def update_pwm2(self,w):
        back = False
        if w < 0:
            back = True
        w = abs(w)
        act_pwm = 0
        """
        if w > self.max_speed[2]:
            act_pwm = 255
            print("wheel 2 cannot go faster")
            rospy.logwarn("wheel 2 cannot go faster")
        elif w < self.min_speed[2]:
            act_pwm = self.min_speed[2]
            print("wheel 2 cannot go this slow")
            rospy.logwarn("wheel 2 cannot this slow")
        else:
            act_pwm = ((255 - self.min_pwm[2])/(self.max_speed[2] - self.min_speed[2])) * w
        if back:
            act_pwm -= act_pwm
        """
        act_pwm = ((255 - self.min_pwm[2])/(self.max_speed[2] - self.min_speed[2])) * w
        if back:
            act_pwm = -act_pwm
        self.pwm[2].data = int(act_pwm)
        print("pwm2 = ",str(self.pwm[2].data))

    def update_pwm3(self,w):
        back = False
        if w < 0:
            back = True
        w = abs(w)
        act_pwm = 0
        """
        if w > self.max_speed[3]:
            act_pwm = 255
            print("wheel 3 cannot go faster")
            rospy.logwarn("wheel 3 cannot go faster")
        elif w < self.min_speed[3]:
            act_pwm = self.min_speed[3]
            print("wheel 3 cannot go this slow")
            rospy.logwarn("wheel 3 cannot this slow")
        else:
            act_pwm = ((255 - self.min_pwm[3])/(self.max_speed[3] - self.min_speed[3])) * w
        if back:
            act_pwm -= act_pwm
        """
        act_pwm = ((255 - self.min_pwm[3])/(self.max_speed[3] - self.min_speed[3])) * w
        if back:
            act_pwm = -act_pwm
        self.pwm[3].data = int(act_pwm)
        print("pwm3 = ",str(self.pwm[3].data))
    #####################################

    def pub_pwm(self, p0,p1,p2,p3):
        self.pwm0.publish(p0)
        self.pwm1.publish(p1)
        self.pwm2.publish(p2)
        self.pwm3.publish(p3)
    
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.self.direction.linear.x = 0
        aux0 = Int32()
        aux0.data = 0

        self.pub_pwm(aux0,aux0,aux0,aux0)
        rospy.logwarn("Remap is dead")
        print("Remap is dead") 

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("Remap", anonymous=True) 
    Remap()