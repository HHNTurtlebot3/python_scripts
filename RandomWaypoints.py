#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def RandomWP():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    # define FiFo
    RandNum=[]
    OrderList=[]

    while len(OrderList) < 5:   
        RandNum = random.randint(1,5)
        if RandNum in OrderList:
            continue
        else:
            OrderList.append(RandNum)

    GoalVectors = {1:[1.8, 0.03], 2:[1.6, 0.2], 3:[1.5 ,0.9], 4:[-1.0, 0.6], 5:[1.0, 0.5]} 
    TargetList = []

    while len(TargetList) < 5:       
        Comp = OrderList.pop()
        if Comp in GoalVectors:
            Target= GoalVectors[Comp]
            TargetList.append(Target)

    TargetList = np.array(TargetList)
    print(TargetList)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # define loop
    for i in range(0,5):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = TargetList[i,0]
        goal.target_pose.pose.position.y = TargetList[i,1]
        goal.target_pose.pose.position.z = -0.001 # fixed z coordinate
        goal.target_pose.pose.orientation.w = 0.09012465928 # fixed angles
        goal.target_pose.pose.orientation.z = 0.098917586 # of robot position for all goals
        print("Aktuelles Ziel x:", goal.target_pose.pose.position.x, "y:", goal.target_pose.pose.position.y)

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server DOWN ;/ ")
            return False
        else:
            print("A Goal is Executed") 
    return True

def mainWP():
    try:
        n = 0
        while n < 1:
            result = RandomWP()
            n +=1
        rospy.loginfo("All Goals executed ")
        return True
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation DONE ")