#!/usr/bin/env python 
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool

class AutonomousControl(): 

    ######### Constructor ##########
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        ####################### SUBSCRIBERS ############################
        rospy.Subscriber("autonomous_control_on", Bool, self.auto_control_cb)
        rospy.Subscriber("scan", LaserScan, self.laser_cb)
        rospy.Subscriber("fire_detection", String, self.fire_detection_cb)
        rospy.Subscriber("turn_robot", Bool, self.turn_cb)

        ####################### PUBLISHERS ############################ 
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
        
        ######################## CONSTANTS AND VARIABLES ############################## 
        
        ##Autonomous control
        self.autonomous_control_on = False

        ##Fire homing
        self.fire_homing_enabled = False
        self.fire_in_water_range = False
        self.fire_angle = np.inf
        self.k_FH0 = 0.6

        #After Spray Turn
        self.turn_robot_enabled = False
        self.turn_robot_counter = 0

        ##Obstacle avoidance
        self.avoid_obstacle_range = 0.9
        self.robot_radius = 0.3
        self.k_A0 = 0.6
        self.closest_angle = 0.0 #Angle to the closest object
        self.closest_range = np.inf #Distance to the closest object

        ##ROS
        self.vel_msg = Twist()
        r = rospy.Rate(10) #10Hz is the lidar's frequency 
        print("Node initialized 1hz")
        ############################### MAIN LOOP #####################################
        while not rospy.is_shutdown():
            
            if self.autonomous_control_on:

                if self.turn_robot_enabled:
                    self.vel_msg = self.turn_robot()
                else:
                    self.vel_msg = self.search_fire(self.vel_msg)
                
                self.vel_msg = self.avoid_obstacles(self.vel_msg)

                self.cmd_vel_pub.publish(self.vel_msg)
                

            r.sleep()

    ############################### METHODS #####################################

    ######### Fire homing  ##########
    def turn_robot(self):
        print("Turning Robot")
        self.vel_msg.linear.x = 0.0 #m/s
        
        if(self.turn_robot_counter < 30):
            self.vel_msg.angular.z = 0.5 #turns 90 degrees (PI/2) in 0.8 seconds (20 cycles at 10Hz)
            self.turn_robot_counter += 1

        else:
            self.vel_msg.angular.z = 0.0
            self.turn_robot_enabled = False
            self.turn_robot_counter = 0

        return self.vel_msg
        
    def stop_robot(self):
        self.vel_msg.linear.x = 0.0 #m/s
        self.vel_msg.angular.z = 0.0 #rad/s
        
        self.cmd_vel_pub.publish(self.vel_msg)
        print("Robot Stopped")

    def search_fire(self, vel_msg):
        print("Searching Fire")
        if self.fire_homing_enabled:
            if abs(self.fire_angle) > 8 and self.fire_angle != np.inf and not self.fire_in_water_range: 
                vel_msg.angular.z = self.k_FH0 * (self.fire_angle) * np.pi / 180
                vel_msg.linear.x = 0.0 #m/s
                return vel_msg

            elif self.fire_in_water_range:
                vel_msg.linear.x = 0.0 #m/s
                vel_msg.angular.z = 0.0 #rad/s
                return vel_msg
             
        vel_msg.linear.x = 0.2 #m/s
        vel_msg.angular.z = 0.0 #rad/s
        return vel_msg

    ######### Obstacle Avoidance  ##########
    def avoid_obstacles(self, vel_msg):
        if self.closest_range <= self.robot_radius:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.0
        elif self.closest_range <= self.avoid_obstacle_range and (abs(self.closest_angle) >= 0 and abs(self.closest_angle) <= math.pi/2):
            theta_A0 = self.closest_angle+np.pi
            vel_msg.angular.z = self.k_A0 * theta_A0

        #print("closest object range: " + str(self.closest_range))
        #print("closest object angle: " + str(self.closest_angle))
        return vel_msg

    ######## LIDAR VALUE FILTERING ########

    ############################### CALLBACKS #####################################
    def auto_control_cb(self, msg):
        if self.autonomous_control_on and not msg.data:
            self.stop_robot()

        self.autonomous_control_on = msg.data
        print("Autonomous Control: {}".format("ON" if self.autonomous_control_on else "OFF"))

    def fire_detection_cb(self, msg):
        fire_data = msg.data.split(",")

        if len(fire_data) == 3:
            self.fire_homing_enabled = True if fire_data[0] == "1" else False
            self.fire_in_water_range = True if fire_data[1] == "1" else False
            self.fire_angle = np.inf if fire_data[2] == "inf" else int(fire_data[2])
        else:
            self.fire_homing_enabled = False
            self.fire_in_water_range = False
            self.fire_angle = np.inf 

    def turn_cb(self, msg):
        self.turn_robot_enabled = msg.data
        print("Turn robot callback" + " True" if self.turn_robot_enabled else "False" )

    def laser_cb(self, msg): 
        ## This function receives a message of type LaserScan and computes the closest object direction and range
        closest_range = np.inf
        ranges = msg.ranges
        for i in range(len(ranges)):
            if ranges[i] < closest_range and ranges[i] > 0:
                closest_range = ranges[i]

        idx = msg.ranges.index(closest_range)
        closest_angle = msg.angle_min + idx * msg.angle_increment
        # Limit the angle to [-pi,pi]
        closest_angle = np.arctan2(np.sin(closest_angle),np.cos(closest_angle))
        self.closest_range = closest_range
        self.closest_angle = closest_angle

    ############################### CLEANUP #####################################    
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("autonomous_control", anonymous=True) 
    AutonomousControl()
