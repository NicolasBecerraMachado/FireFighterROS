#!/usr/bin/env python 
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist
from geometry_msgs.msg import String
from std_msgs.msg import Bool

class AutonomousControl(): 

    ######### Constructor ##########
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) 

        ####################### SUBSCRIBERS ############################
        rospy.Subscriber("autonomous_control_on", Bool, self.auto_control_cb)
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)
        rospy.Subscriber("fire_detection", String, self.fire_detection_cb)
        

        ####################### PUBLISHERS ############################ 
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
        
        ######################## CONSTANTS AND VARIABLES ############################## 
        
        ##Autonomous control
        self.autonomous_control_on = False
        self.stop_robot = False

        ##Fire homing
        self.fire_homing_enabled = False
        self.fire_in_water_range = False
        self.fire_angle = np.inf
        self.k_FH0 = 0.6

        ##Obstacle avoidance
        self.avoid_obstacle_range = 1.0
        self.robot_radius = 0.3
        self.k_A0 = 0.8
        self.closest_angle = 0.0 #Angle to the closest object
        self.closest_range = np.inf #Distance to the closest object

        ##ROS
        self.vel_msg = Twist()
        r = rospy.Rate(10) #10Hz is the lidar's frequency 
        print("Node initialized 1hz")

        ############################### MAIN LOOP #####################################
        while not rospy.is_shutdown():
            
            if self.autonomous_control_on:

                vel_msg = search_fire(vel_msg)
                vel_msg = avoid_obstacles(vel_msg)
                
                self.cmd_vel_pub.publish(vel_msg)

            elif self.stop_robot:

                vel_msg.linear.x = 0.0 #m/s
                vel_msg.angular.z = 0.0 #rad/s
                
                self.cmd_vel_pub.publish(vel_msg)
                self.stop_robot = False

            r.sleep()

    ############################### METHODS #####################################

    ######### Fire homing  ##########
    def search_fire(self, vel_msg):
        if self.fire_homing_enabled:
            if abs(self.fire_angle) > 10 and self.fire_angle != np.inf: 
                vel_msg.angular.z = self.k_FH0 * (self.fire_angle) * np.pi / 180
                return vel_msg

            elif self.fire_in_water_range:
                vel_msg.linear.x = 0.0 #m/s
                vel_msg.angular.z = 0.0 #rad/s
                return vel_msg
             
        vel_msg.linear.x = 0.3 #m/s
        vel_msg.angular.z = 0.0 #rad/s
        return vel_msg

    ######### Obstacle Avoidance  ##########
    def avoid_obstacles(self, vel_msg):
        if self.closest_range <= robot_radius:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.0
        elif self.closest_range <= avoid_obstacle_range and (abs(self.closest_angle) >= 0 and abs(self.closest_angle) <= math.pi/2):
            theta_A0 = self.closest_angle+np.pi
            vel_msg.angular.z = k_A0 * theta_A0

        #print("closest object range: " + str(self.closest_range))
        #print("closest object angle: " + str(self.closest_angle))

    ######## LIDAR VALUE FILTERING ########
    def filter_lidar_zeros(ranges):
        for i in range(ranges):
            if ranges[i] <= 0:
                ranges[i] = np.inf
        return ranges

    ############################### CALLBACKS #####################################
    def auto_control_cb(self, msg):
        if self.autonomous_control_on and not msg.data:
            self.stop_robot = True

        self.autonomous_control_on = msg.data

    def fire_detection_cb(self, msg):
        fire_data = msg.data.split(",")

        if len(fire_data) > 0:
            self.fire_homing_enabled = True if fire_data[0] == "1" else False
            self.fire_in_water_range = True if fire_data[1] == "1" else False
            self.fire_angle = int(fire_data[2]) if fire_data[2] == "inf" else np.inf
        else:
            self.fire_homing_enabled = False
            self.fire_in_water_range = False
            self.fire_angle = np.inf 

    def laser_cb(self, msg): 
        ## This function receives a message of type LaserScan and computes the closest object direction and range
        ranges = filter_lidar_zeros(msg.ranges)
        closest_range = min(ranges)
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