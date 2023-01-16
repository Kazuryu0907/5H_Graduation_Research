#! /usr/bin/env python3

from logging.config import listen
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,PoseStamped
import math
import time
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import actionlib

scan = LaserScan()
vel = PoseStamped()
def callback(msg:LaserScan):
    scan.ranges = msg.ranges
    scan.angle_min = msg.angle_min
    scan.angle_max = msg.angle_max
    scan.angle_increment = msg.angle_increment
    scan.range_min = msg.range_min
    scan.range_max = msg.range_max
    #print(f"range ahead:{range_aheard:.1f}")

rospy.init_node("range_ahead")
scan_sub = rospy.Subscriber("scan",LaserScan,callback)

rate = rospy.Rate(10)

preflont = float("inf")

def pub_x():
    pub = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist,queue_size=10)
    dist = 4.5 #m
    speed = 1 #m/s
    tar_time = dist / speed
    t = Twist()
    t.linear.x = speed
    t.angular.z = 0

    s_time = time.time()
    e_time = time.time()
    rate = rospy.Rate(30)
    while e_time - s_time <= tar_time:
        pub.publish(t)
        e_time = time.time()
        rate.sleep()

def goal_pose():
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'odom'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose.position.x = 4.5
    goal_pose.target_pose.pose.orientation.w = 1.

    return goal_pose

def send():
    client = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    client.wait_for_server()
    goal = goal_pose()
    client.send_goal(goal)
    client.wait_for_result()
    print(client.get_state())

while not rospy.is_shutdown():
    #rospy.loginfo(f"Rays:{len(scan.ranges)}")
    #rospy.loginfo(f"angle[rad] Min:{scan.angle_min} Max:{scan.angle_max}")
    #165~195
    if len(scan.ranges) is 0:
        continue
    # flont = scan.ranges[175*2:185*2]
    flont = [scan.ranges[360]]
    flont_intensities = min(flont)
    if flont_intensities > 5.:
        flont_intensities = float("inf")
    print(flont_intensities)
    if math.isinf(flont_intensities) and not math.isinf(preflont):
        rospy.loginfo("Go")
        send()
        # vel_pub.publish(goal)

    preflont = flont_intensities
    rate.sleep()