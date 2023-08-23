#!/usr/bin/env python

# Implementation and testing of algorithms for autonomous navigation on an experimental omnidirectional
# mobile robotic platform in a Mecanum wheel configuration

# University of Costa Rica
# Author: Luis Javier Herrera Barrantes

# Description:
# Braitenberg obstacle avoidance algorithm and path tracking Decentralized Point 
# implemented for the real robotic platform using ROS.



import rospy
import math

from std_msgs.msg import Int8
from mecanumrob_common.msg import EncTimed, WheelSpeed
import csv

from rospy.client import get_param
from sensor_msgs.msg import LaserScan
import numpy as np


Tiempo = 0.023

# Parameters
k=1
ly= 0.17
lx= 0.125

# Braitenberg parameters
FRONT_DISTANCE = 1.5
TURN_DISTANCE = 1.5
TURN_DISTANCE2 = 0.8
DELTA_TURN  = 2

MAX_DRIVE_SPEED = 30


class TrajectoryFollower():

    def __init__(self, path_file):
        self.base_name = rospy.get_param("~BaseName", default="")
        self.path = []
        self.path_index = 0
        self.e = 0.1         # Distance from the reference point to the traction axis
        self.b = 0.352/2     # Wheelbase

        # Odometry write results in file
        self.f = open("resultados.csv", "w")
        self.writer = csv.writer(self.f)

        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0

 
        self.load_path(path_file)

        self.m1_pwm_pub = rospy.Publisher("%s/motor/NE_pwm" % self.base_name, Int8, queue_size=0)
        self.m2_pwm_pub = rospy.Publisher("%s/motor/NW_pwm" % self.base_name, Int8, queue_size=0)
        self.m3_pwm_pub = rospy.Publisher("%s/motor/SW_pwm" % self.base_name, Int8, queue_size=0)
        self.m4_pwm_pub = rospy.Publisher("%s/motor/SE_pwm" % self.base_name, Int8, queue_size=0)

        
        self.enc_sub =  rospy.Subscriber("%s/wheel_speed"  % self.base_name, WheelSpeed,  self.update_encoders_speed, queue_size = 100) #Read velocity of encoder

    def load_path(self, path_file):
        with open(path_file, 'r') as csvfile:
            path_reader = csv.reader(csvfile)
            for row in path_reader:
                x = float(row[0])
                y = float(row[1])
                self.path.append([x, y])

    def update_encoders_speed(self, encoders):

        # Apply direct kinematic model for Mecanum wheels
        v_enc = np.array([encoders.phi[0],encoders.phi[1], encoders.phi[2],encoders.phi[3]])
        vel_ang_encoder = np.transpose(v_enc)

        rot = (0.045/4)*np.array([[math.cos(self.theta), (-1)*math.sin(self.theta), 0],
                                 [math.sin(self.theta), math.cos(self.theta), 0],
                                 [0,0,1]],dtype=np.float64)

        J = np.array([[1,1,1,1],
                     [-1, 1, -1,1],
                     [ 1/(lx+ly),-1/(lx+ly),-1/(lx+ly),1/(lx+ly)]],dtype=np.float64)

        result_matrix1 = np.matmul(rot, J)

        v_vector_direct = np.matmul(result_matrix1, vel_ang_encoder)
        D_x = v_vector_direct[0]*Tiempo
        D_y = ((v_vector_direct[1])*Tiempo)
        self.pos_x = self.pos_x + D_x
        self.pos_y = (self.pos_y + D_y)
        print(v_vector_direct)

       

        ####################### Descentrilized Point #################################
        # Gains
        kvx = 0.7*k
        kvy = 0.7*k
        kpx = 1.5*k
        kpy = 1.5*k


        # Obtain actual orientation of the robot
        theta = self.theta

        # Obtain actual position of the robot
        xp = self.pos_x
        yp = self.pos_y

        # Get the desired position at the next reference point
        xref = self.path[self.path_index][0]
        yref = self.path[self.path_index][1]

        # Calculate diference in position
        dx = xref - xp
        dy = yref - yp

        # Robot kinematic control
        xpunto = kvx*dx + kpx*(xref - (xp + self.e*math.cos(theta)))
        ypunto = kvy*dy + kpy*(yref - (yp + self.e*math.sin(theta)))

        
        # Apply inverse kinematic model for Mecanum wheels
        v_vector= np.array([xpunto,ypunto,v_vector_direct[2]])

        v_vector = np.transpose(v_vector)   # Column vector

 
        H = (1/0.045)*np.array([[1,-1,-(lx+ly)],[1,1,(lx+ly)],[1,1,-(lx+ly)],[1,-1,(lx+ly)]], dtype=np.float64)

        wheel_vel_vector = np.matmul(H,v_vector)
       

        # Intensity values
        vfi_Intensidad = float(2.7*wheel_vel_vector[0]) #100  
        vfd_Intensidad = float(2.7*wheel_vel_vector[1]) #100
        vai_Intensidad = float(2.7*wheel_vel_vector[2]) #100  
        vad_Intensidad = float(2.7*wheel_vel_vector[3]) #100

        VELfi = vfi_Intensidad
        VELfd = vfd_Intensidad
        VELai = vai_Intensidad
        VELad = vad_Intensidad

       # Print info
        print('vfi {} vfd {} vai {} vad {}'.format(vfi_Intensidad, vfd_Intensidad, vai_Intensidad, vad_Intensidad))
        print(' x {}  y {}'.format(self.pos_x, self.pos_y))
        row = (self.pos_x, self.pos_y)
        self.writer.writerow(row)
    
        
       # Publish current values ​​to each robot motor
        if self.pos_x <= 3 and self.pos_y <= 1:
            self.m1_pwm_pub.publish(Int8(-60))
            self.m2_pwm_pub.publish(Int8(60))
            self.m3_pwm_pub.publish(Int8(60))
            self.m4_pwm_pub.publish(Int8(-60))

        elif self.pos_y < 3:
            self.m1_pwm_pub.publish(Int8(60))
            self.m2_pwm_pub.publish(Int8(60))
            self.m3_pwm_pub.publish(Int8(-60))
            self.m4_pwm_pub.publish(Int8(-60))

        elif self.pos_y > 3:
            self.m1_pwm_pub.publish(Int8(60))
            self.m2_pwm_pub.publish(Int8(-60))
            self.m3_pwm_pub.publish(Int8(-60))
            self.m4_pwm_pub.publish(Int8(60))
 
       
        detener = False
        if self.pos_y >= 2 and detener == True:
            self.Stop()
        else:
            self.m1_pwm_pub.publish(Int8(-VELad))
            self.m2_pwm_pub.publish(Int8(VELai)) #VELi
            self.m3_pwm_pub.publish(Int8(VELfi))
            self.m4_pwm_pub.publish(Int8(-VELfd))
       
        # Check if the robot has reached the waypoint and update the index of the next waypoint
        if abs(dx) < 0.4 and abs(dy) < 0.4:
            self.path_index += 1
            if self.path_index >= len(self.path):
                self.f.close()
                self.Stop()
                rospy.loginfo("Trayectoria completada!")
                rospy.signal_shutdown("Trayectoria completada")

                

        print('---------------------------------')

        return None 

    # Stop robot
    def Stop(self):
        self.m1_pwm_pub.publish(Int8(0))
        self.m2_pwm_pub.publish(Int8(0))
        self.m3_pwm_pub.publish(Int8(0))
        self.m4_pwm_pub.publish(Int8(0))





class BraitenbergMod(object):
    def __init__(self):
        self.dir = ""
        self.base_name = rospy.get_param("~BaseName", default="")

        # Publishers:
        self.m1_pwm_pub = rospy.Publisher("%s/motor/NE_pwm" % self.base_name, Int8, queue_size=0)
        self.m2_pwm_pub = rospy.Publisher("%s/motor/NW_pwm" % self.base_name, Int8, queue_size=0)
        self.m3_pwm_pub = rospy.Publisher("%s/motor/SW_pwm" % self.base_name, Int8, queue_size=0)
        self.m4_pwm_pub = rospy.Publisher("%s/motor/SE_pwm" % self.base_name, Int8, queue_size=0)

        # Subscribers:
        self.v_i_Intensidad = MAX_DRIVE_SPEED/2
        self.v_d_Intensidad = MAX_DRIVE_SPEED/2

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=100) #####
   
    def scan_callback(self, scan_msg):
        scan_measures = scan_msg.ranges
        print("-------------------------------------------------")

        # Splits ranges in n:
        LR = np.array_split(scan_measures, 12)#n)     # L and R split (n Zones)

        # Separation on L and R avg measures:
        front_zone = 7
        R1_zone = 6
        L1_zone = 8
        R2_zone = 5
        L2_zone = 9

        front = 0
        R1 = 0
        L1 = 0
        R2 = 0
        L2 = 0

        # Counters
        f_i = 0.01
        r1_i = 0.01
        l1_i = 0.01
        r2_i = 0.01
        l2_i = 0.01

        for i in range(0,len(LR[front_zone])):
            if not LR[front_zone][i] > 20:
                front = front + LR[front_zone][i]
                f_i += 1

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

        print(l1_i)
        front = round(front/f_i, 6)
        R1 = round(R1/r1_i, 6)
        L1 = round(L1/l1_i, 6)
        R2 = round(R2/r2_i, 6)
        L2 = round(L2/l2_i, 6)


        print("Intensidad i {}    Intensidad d {}".format(self.v_i_Intensidad,self.v_d_Intensidad))
        print("-------------------------------------------------")


        # -------------- GOOOOOOO: -----------------------  
        #   +steering: left | -steering: rigth
        if(front < FRONT_DISTANCE):
            self.v_i_Intensidad = self.v_i_Intensidad - DELTA_TURN
            self.v_d_Intensidad = self.v_d_Intensidad + DELTA_TURN
            if self.v_i_Intensidad <= 0:
                self.v_i_Intensidad = 0
            if self.v_d_Intensidad >= MAX_DRIVE_SPEED:
                self.v_d_Intensidad = MAX_DRIVE_SPEED
 

        elif(L1 < TURN_DISTANCE or L2 < TURN_DISTANCE2):

            self.v_i_Intensidad = self.v_i_Intensidad + DELTA_TURN
            self.v_d_Intensidad = self.v_d_Intensidad - DELTA_TURN

            if self.v_d_Intensidad <= 0:
                self.v_d_Intensidad = 0
            if self.v_i_Intensidad >= MAX_DRIVE_SPEED:
                self.v_i_Intensidad = MAX_DRIVE_SPEED
            
            print("\t\t\t\t\tTurning R")

        elif(R1 <= TURN_DISTANCE or R2 <= TURN_DISTANCE2):

            self.v_i_Intensidad = self.v_i_Intensidad - DELTA_TURN
            self.v_d_Intensidad = self.v_d_Intensidad + DELTA_TURN

            if self.v_i_Intensidad <= 0:
                self.v_i_Intensidad = 0
            if self.v_d_Intensidad >= MAX_DRIVE_SPEED:
                self.v_d_Intensidad = MAX_DRIVE_SPEED
            
            print("\tTurning L")
            

        else:
            self.v_i_Intensidad = MAX_DRIVE_SPEED
            self.v_d_Intensidad = MAX_DRIVE_SPEED
            VELi = self.v_i_Intensidad
            VELd = self.v_d_Intensidad
            self.m1_pwm_pub.publish(Int8(-VELd))
            self.m2_pwm_pub.publish(Int8(VELi))
            self.m3_pwm_pub.publish(Int8(VELi))
            self.m4_pwm_pub.publish(Int8(-VELd))
            self.v_i_Intensidad = MAX_DRIVE_SPEED/2
            self.v_d_Intensidad = MAX_DRIVE_SPEED/2
            print('Prueba')
            return 0

        VELi = self.v_i_Intensidad
        VELd = self.v_d_Intensidad
        self.m1_pwm_pub.publish(Int8(-VELd))
        self.m2_pwm_pub.publish(Int8(VELi))
        self.m3_pwm_pub.publish(Int8(VELi))
        self.m4_pwm_pub.publish(Int8(-VELd))

        return 0




if __name__ == '__main__':
    try:
        rospy.init_node("RoboclawTester", log_level=rospy.DEBUG)
        #trajectory_follower = TrajectoryFollower(path_file="/home/dif3/catkin_workspace/src/MiniMecanum/mecanumrob_roboclaw/scripts/ruta_circulo.csv")
        brai = BraitenbergMod()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

