#! /usr/bin/env python3

import time

from matplotlib.colors import LinearSegmentedColormap
import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Twist
import math
from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseGoal,MoveBaseAction


p = PoseStamped()

def callback(data:MoveBaseActionGoal):
    pos = data.goal.target_pose.pose
    print("[({0},{1},0.0),(0.0,0.0,{2},{3})],".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w))

def listener():

    rospy.init_node('goal_sub', anonymous=True)

    rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback)
    rospy.spin()

def goal_pose():
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'odom'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose.position.x = 4.5
    goal_pose.target_pose.pose.orientation.w = 1.

    return goal_pose

def send():
    rospy.init_node("patrol")
    client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    client.wait_for_server()
    goal = goal_pose()
    client.send_goal(goal)
    client.wait_for_result()
    print(client.get_state())
send()
    