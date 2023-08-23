#!/usr/bin/env python3

# Implementation and testing of algorithms for autonomous navigation on an experimental omnidirectional
# mobile robotic platform in a Mecanum wheel configuration

# University of Costa Rica
# Author: Luis Javier Herrera Barrantes


# Description:
# Algorithm for the combination of behaviors between trajectory tracking algorithm Decentralized Point
# and obstacle avoidance Braitenberg implemented for the simulation in Gazebo and using ROS.



from pp_utils2 import get_actuation, load_waypoints, get_current_waypoint
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf
import math
import csv
from tf.transformations import euler_from_quaternion

path_file="/home/cerlabrob/Documents/Rutas/cuadrado2.csv"

# Parameters sum of weights
GSW = 1 # "global_speed_weight"
LSW  = 0 # "local_speed_weight"
GSTRW = 1 # "global_steer_weight"
LSTRW  = 13 # 0.85 # "local_steer_weight"


# Braitenberg Parameters
TURN_DISTANCE = 1
TURN_DISTANCE2 = 0.5
DELTA_TURN  = 2
DELTA_SPEED = 2
MAX_DRIVE_SPEED = 0.4
MAX_TURN_SPEED = 0.4
 
n = 12

class PathPlanningNode():
    def __init__(self):
        
        # Write results in csv file
        self.f = open('resultados_e0.45_kv0.2_kp1.5.csv', 'w')
        self.writer = csv.writer(self.f)

        self.e = 0.45 # Distance from the reference point to the traction axis
        self.b = 0.2 # Wheelbase
        self.path_index = 0

        self.xp = 0
        self.yp = 0
        
        self.speed = 0
        self.speed_W = 0 
        
        self.dx = 0
        self.dy = 0

        self.waypoints = load_waypoints()
        self.path = self.load_path()
        
        # Braitenberg part 
        
        self.current_angular_v = 0.0
        self.speedBrai = MAX_DRIVE_SPEED
        
        self.drive = Twist()
        self.drive_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.drive_handler, queue_size=100)
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=100)
        
    def load_path(self):
        path = []
        with open(path_file, 'r') as csvfile:
            path_reader = csv.reader(csvfile)
            #next(path_reader) # skip header
            for row in path_reader:
                x = float(row[0])
                y = float(row[1])
                path.append([x, y])

        return path 

            
        
    
    def drive_handler(self, odom_msg):
    
        ########################### Decentrilized Point ##############################################
        
        # Gains
        kvx = 0.2
        kvy = 0.2
        kpx = 1.5
        kpy = 1.5


        # Obtain actual orientation of the robot
        orientation = odom_msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Obtain actual position of the robot
        self.xp = odom_msg.pose.pose.position.x
        self.yp = odom_msg.pose.pose.position.y


        # Get the desired position at the next reference point
        xref = self.path[self.path_index][0]
        yref = self.path[self.path_index][1]


        # Calculate diference in position
        self.dx = xref - self.xp
        self.dy = yref - self.yp

        # Robot kinematic control
        xpunto = kvx*self.dx + kpx*(xref - (self.xp + self.e*math.cos(theta)))
        ypunto = kvy*self.dy + kpy*(yref - (self.yp + self.e*math.sin(theta)))

        
        # Inverse kinematics
        ecos_theta = self.e * math.cos(theta)
        esin_theta = self.e * math.sin(theta)
        bcos_theta = self.b * math.cos(theta)
        bsin_theta = self.b * math.sin(theta)

        # Velocity relations
        vr = (((ecos_theta - bsin_theta) * xpunto) + ((esin_theta + bcos_theta) * ypunto)) / self.e
        vl = (((ecos_theta + bsin_theta) * xpunto) + ((esin_theta - bcos_theta) * ypunto)) / self.e

        self.speed = (vr + vl) / 2.0
        self.speed_W = ((vr - vl) / (2.0 * self.b))

        
        
        # ################# Braitenberg ############################
        self.current_angular_v  = odom_msg.twist.twist.angular.z
        
        
        return 0
        
        
    def scan_callback(self, scan_msg):

        # NOTE: scan measurements are taken in counter-clockwize direction
        scan_measures = scan_msg.ranges           # 22.5º each, zones to split scan measurements

        row = (self.xp,self.yp)
        self.writer.writerow(row)
        # Splits ranges in n:
        print("Current x: {}    Current y: {}".format(self.xp, self.yp))
        LR = np.array_split(scan_measures, n)     # L and R split (n Zones)


        # Separation on L and R avg measures:
        R1_zone = 11
        L1_zone = 0
        R2_zone = 10
        L2_zone = 1
        
        # Initialize 
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
     
        
        # ######### Combination ######################################
        
        # Combination parameters
        self.drv_global_speed = self.speed
        self.drv_global_steering = self.speed_W
        self.drv_local_speed = self.speedBrai
        self.drv_local_steering = self.current_angular_v
        
        print(' ')
        print(' ')
        
        # Suma of weights
        speed_overall = GSW*self.drv_global_speed + LSW*self.drv_local_speed
        steer_overall = GSTRW*self.drv_global_steering + LSTRW*self.drv_local_steering
        
        self.drive.linear.x = speed_overall
        self.drive.angular.z = steer_overall
        
        self.drive_pub.publish(self.drive)


        # Check if the robot has reached the waypoint and update the index of the next waypoint
        if abs(self.dx) < 0.8 and abs(self.dy) < 0.8:
            self.path_index += 1
            if self.path_index >= len(self.path):
                self.f.close()
                rospy.loginfo("Trayectoria completada!")
                rospy.signal_shutdown("Trayectoria completada")



        print("VELOCIDAD LINEAL: {}    RADIAL: {}".format(speed_overall, steer_overall))
        print("VELOCIDADL LINEAL: {}    RADIAL: {}".format(self.drv_local_speed, self.drv_local_steering))
        print("VELOCIDADG LINEAL: {}    RADIAL: {}".format(self.drv_global_speed, self.drv_global_steering))

        return 0

###########################################################################



def main():
    rospy.init_node("path_planner_node")

    pathPlanningNode = PathPlanningNode()
 

    rospy.spin()

if __name__ == '__main__':
    main()

