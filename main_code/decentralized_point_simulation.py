#!/usr/bin/env python3

# Implementation and testing of algorithms for autonomous navigation on an experimental omnidirectional
# mobile robotic platform in a Mecanum wheel configuration

# University of Costa Rica 
# Author: Luis Javier Herrera Barrantes


# Description:
#  Trajectory tracking algorithm implemented for the simulation in Gazebo using ROS

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import csv
import tf
import math
from tf.transformations import euler_from_quaternion


class TrajectoryFollower:
    def __init__(self, path_file):
        rospy.init_node('trajectory_follower', anonymous=True)

        # Write results in .csv file
        self.f = open('resultados_1.csv', 'w')
        self.writer = csv.writer(self.f)

        self.path = []
        self.path_index = 0
        self.e = 0.55 # Distance from the reference point to the traction axis
        self.b = 0.2 # Wheelbase
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.load_path(path_file)

    def load_path(self, path_file):
        with open(path_file, 'r') as csvfile:
            path_reader = csv.reader(csvfile)
            for row in path_reader:
                x = float(row[0])
                y = float(row[1])
                self.path.append([x, y])

    def odom_callback(self, msg):
        

        # Gains
        kvx = 0.8
        kvy = 0.8
        kpx = 1.5
        kpy = 1.5


        # Obtain actual orientation of the robot
        orientation = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Obtain actual position of the robot
        xp = msg.pose.pose.position.x
        yp = msg.pose.pose.position.y

        # Get the desired position at the next reference point
        row = (xp,yp)
        self.writer.writerow(row)
        print(row)

        # Get the desired position at the next reference point
        xref = self.path[self.path_index][0]
        yref = self.path[self.path_index][1]

        # Calculate diference in position
        dx = xref - xp
        dy = yref - yp

        # Robot kinematic control
        xpunto = kvx*dx + kpx*(xref - (xp + self.e*math.cos(theta)))
        ypunto = kvy*dy + kpy*(yref - (yp + self.e*math.sin(theta)))

        
        # Inverse kinematics
        ecos_theta = self.e * math.cos(theta)
        esin_theta = self.e * math.sin(theta)
        bcos_theta = self.b * math.cos(theta)
        bsin_theta = self.b * math.sin(theta)

        # Velocity ratio  
        vr = (((ecos_theta - bsin_theta) * xpunto) + ((esin_theta + bcos_theta) * ypunto)) / self.e
        vl = (((ecos_theta + bsin_theta) * xpunto) + ((esin_theta - bcos_theta) * ypunto)) / self.e


        # Post wheel speeds to the /cmd_vel topic
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = (vr + vl) / 2.0
        cmd_vel_msg.angular.z = (vr - vl) / (2.0 * self.b)
        self.pub_cmd_vel.publish(cmd_vel_msg)


        # Check if the robot has reached the waypoint and update the index of the next waypoint
        if abs(dx) < 0.5 and abs(dy) < 0.5:
            self.path_index += 1
            if self.path_index >= len(self.path):
                self.f.close()
                rospy.loginfo("Trayectoria completada!")
                rospy.signal_shutdown("Trayectoria completada")

if __name__ == '__main__':
    try:
        trajectory_follower = TrajectoryFollower(path_file="/home/cerlabrob/catkin_ws/src/nexus_4wd_mecanum_gazebo/scripts/cuadrado2.csv")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
