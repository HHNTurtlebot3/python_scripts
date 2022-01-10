#!/usr/bin/env python
# -*- coding: utf-8 -*-
from pickle import TRUE
import rospy
from geometry_msgs.msg import Twist
import math

def rotation():
    # define new Node
    # rospy.init_node('Rotate_Roboter', anonymous=True)
    # define Publisher
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print('Suche nach Personen im Umkreis')
    # speed = input("Input your speed (degrees/sec):")
    # angle = input("Type your distance (degrees):")

    speed = 30
    angle = 30

    #Converting from angles to radians
    angular_speed = speed*math.pi/180
    relative_angle = angle*math.pi/180

    #We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    # rospy.sleep(1)
    vel_msg.angular.z = abs(angular_speed)

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    # Forcing our robot to stop
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

    return True

def mainRotation():
    try:
        rotation()
        rospy.loginfo('Drehung um Z abgeschlossen')
        return True
    except rospy.ROSInterruptException:
        pass

# mainRotation()