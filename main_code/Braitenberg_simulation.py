#! /usr/bin/python3


# Implementation and testing of algorithms for autonomous navigation on an experimental omnidirectional
# mobile robotic platform in a Mecanum wheel configuration

# University of Costa Rica 
# Author: Luis Javier Herrera Barrantes

# Description:
# Braitenberg obstacle avoidance algorithm implemented for the Gazebo simulation and using ROS.


import rospy
import numpy as np

from geometry_msgs.msg import Twist  #  from ackermann_msgs.msg import AckermannDriveStamped
from rospy.client import get_param
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tf
import csv

# Catalunya: 
TURN_DISTANCE = 0.8 # 0.8
TURN_DISTANCE2 = 0.5 # 0.5
DELTA_TURN  = 0.005 # 0.05
DELTA_SPEED = 0.01 # 0.01

MAX_DRIVE_SPEED = 0.2
MAX_TURN_SPEED = 0.2
n = 12

class BraitenbergMod(object):
    def __init__(self):
        self.dir = ""
        self.drive = Twist()

        self.f = open('Turn1_0.8_Turn2_0.5_DTurn_0.005_DSpeed_0.01_4.csv', 'w')
        self.writer = csv.writer(self.f)

        # Publishers:
        self.drive_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

        # Subscribers:
        self.current_heading    =   0.0
        self.current_angular_v = 0.0
        self.speed = MAX_DRIVE_SPEED
        self.odom_sub   =   rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=100)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=100)

    def odom_callback(self, odom_msg):

        orientation_list = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                            odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        
        (roll, pitch, theta) = tf.transformations.euler_from_quaternion(orientation_list)
        
        self.current_heading = theta
        self.current_angular_v = odom_msg.twist.twist.angular.z
        row = (odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)
        self.writer.writerow(row)
        
        return 0
   
    def scan_callback(self, scan_msg):

        # NOTE: scan measurements are taken in counter-clockwize direction
        scan_measures = scan_msg.ranges                            # 22.5º each, zones to split scan measurements
        print("-------------------------------------------------")

        # Splits ranges in n:
        LR = np.array_split(scan_measures, n)     # L and R split (n Zones)
    

        R1_zone = 11
        L1_zone = 0
        R2_zone = 10
        L2_zone = 1
        
        # Initialize zones
        R1 = 0
        L1 = 0
        R2 = 0
        L2 = 0

        # Counters
        r1_i = 0.01
        l1_i = 0.01
        r2_i = 0.01
        l2_i = 0.01

      
        for i in range(0,len(LR[R1_zone])):

            if not LR[R1_zone][i] > 20:
                R1 = R1 + LR[R1_zone][i]
                r1_i += 1

            if not LR[L1_zone][i] > 20:
                L1 = L1 + LR[L1_zone][i]
                l1_i += 1

            if not LR[R2_zone][i] > 20:
                R2 = R2 + LR[R2_zone][i]
                r2_i += 1

            if not LR[L2_zone][i] > 20:
                L2 = L2 + LR[L2_zone][i]
                l2_i += 1

        R1 = round(R1/r1_i, 6)
        L1 = round(L1/l1_i, 6)
        R2 = round(R2/r2_i, 6)
        L2 = round(L2/l2_i, 6)

        if R1 == 0:
            R1 = float('inf')

        if L1 == 0:
            L1 = float('inf')

        if R2 == 0:
            R2 = float('inf')

        if L2 == 0:
            L2 = float('inf')

        # Center = scan_measures[540] Problema con el rango
        Center = scan_measures[int((360)/2)]
        
        # -------------- GOOOOOOO: -----------------------  
        #   +steering: left | -steering: rigth
        if(L1 < TURN_DISTANCE or L2 < TURN_DISTANCE2):

            self.current_angular_v = (self.current_angular_v - DELTA_TURN)
            self.speed = (self.speed - DELTA_SPEED)
            if self.current_angular_v <= -MAX_TURN_SPEED:
                self.current_angular_v = -MAX_TURN_SPEED
            
            print("\t\t\t\t\tTurning R")

        elif(R1 <= TURN_DISTANCE or R2 <= TURN_DISTANCE2):
 
            self.current_angular_v = (self.current_angular_v + DELTA_TURN)
            self.speed = (self.speed - DELTA_SPEED)
            if self.current_angular_v >= MAX_TURN_SPEED:
                self.current_angular_v = MAX_TURN_SPEED
            
            print("\tTurning L")
            

        else:
            self.current_angular_v = 0
            self.speed = MAX_DRIVE_SPEED
        
        if self.speed <= 0:
            self.speed = 0


        print("\n L1: \t",  L1,
            "\n R1: \t", R1,
            "\n L2: \t",  L2,
            "\n R2: \t", R2)

        self.drive.angular.z = self.current_angular_v
        self.drive.linear.x = self.speed

        print("Velocidad lineal {}    Velocidad angular {}".format(self.speed,self.current_angular_v))
        print("-------------------------------------------------")

        # Publishing drive msgs:
        self.drive_pub.publish(self.drive)
        return 0


def main():
    rospy.init_node('braitenbergPlus_node')
    braitenberg_mod = BraitenbergMod()
    rospy.spin()

if __name__ == '__main__':
    main()


    

