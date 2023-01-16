#!/usr/bin/env python3
import rospy
import numpy as np
from open_mpc_controller.msg import OptimizationParameters,OptimizationResult
from geometry_msgs.msg import Twist
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as T
from obstacle_detector.msg import Obstacles,CircleObstacle
from scipy.stats import vonmises

np.set_printoptions(precision=3)

msg = OptimizationParameters()
Nobs = 10
Nnear = 2
x = np.array([0,0,0.])
x_ref = np.array([15.,0,0])
obs = np.array([[100,100,0] for _ in range(Nobs)])
obs_vel = np.array([[0,0,0] for _ in range(Nobs)])
obs_raidus = np.array([0 for _ in range(Nobs)])
obs_near = np.array([[100,100,0] for _ in range(Nnear)])
obs_near_vel = np.array([[0,0,0] for _ in range(Nnear)])

def callback(msg:OptimizationResult):
    lin_vel,ang_vel = msg.solution[:2]
    twist = Twist()
    twist.linear.x = lin_vel
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = ang_vel
    pub_vel.publish(twist)
    rospy.loginfo(f"v:{msg.solution[0]:.1f} w:{msg.solution[1]:.1f} time:{msg.solve_time_ms:.1f}")
    # rospy.loginfo(f"u:{msg.solution[:2]}")

def obs_callback(msg:Obstacles):
    global obs,obs_vel,obs_raidus
    _obs = np.empty((0,3))
    _obs_vel = np.empty((0,3))
    _obs_raidus = np.empty((0,1))
    obss = msg.circles
    for i,o in enumerate(obss):
        # if o.radius <= 0.4:
        #     continue
        circle = CircleObstacle()
        circle.center = o.center
        _obs = np.append([[o.center.x,o.center.y,0]],_obs,axis=0)
        circle.velocity = o.velocity
        _obs_vel = np.append([[o.velocity.x*100,o.velocity.y*100,0]],_obs_vel,axis=0)
        circle.radius = o.radius
        _obs_raidus = np.append(o.radius,_obs_raidus)
        circle.true_radius = o.true_radius
    #wall
    # _obs = np.concatenate([_obs,np.array([[i*2+1,4,0] for i in range(3)]),np.array([[i*2+1,-4,0] for i in range(3)])])
    # _obs_vel = np.concatenate([_obs_vel,np.array([[0,0,0] for i in range(6)])])
    if len(_obs) < Nobs:
        complement = Nobs - len(_obs)
        _obs = np.concatenate([_obs,np.array([[1000,1000,0] for _ in range(complement)])])
        _obs_vel = np.concatenate([_obs_vel,np.array([[0,0,0] for _ in range(complement)])])
    obs = _obs.copy()
    obs_vel = _obs_vel.copy()
    # rospy.loginfo(obs)
rospy.init_node("ppp",anonymous=True)
pub = rospy.Publisher("/open_mpc_controller/parameters",OptimizationParameters,queue_size=10)
sub = rospy.Subscriber("/open_mpc_controller/result",OptimizationResult,callback)
pub_vel = rospy.Publisher("/husky_velocity_controller/cmd_vel",Twist,queue_size=10)
raw_obstacle = rospy.Subscriber("/obs",Obstacles,obs_callback)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)


# params = np.concatenate([x,x_ref,obs,obs_vel,obs_near,obs_near_vel])
# msg.parameter = [0.0,0.0,0.0,8.0,10.0,0.0,5.0,2.5,0.0,5.0,3.0,0.0,5.0,3.5,0.0,5.0,4.0,0.0,5.0,4.5,0.0,5.0,5.0,0.0,5.0,5.5,0.0,5.0,6.0,0.0,5.0,6.5,0.0,5.0,6.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.7,-0.7,0.0,5.0,2.5,0.0,5.0,3.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
# msg.parameter = [0,1,1,1,1]
# msg.parameter = params
msg.initial_guess = [0.]*20
msg.initial_penalty = 1.0
rate = rospy.Rate(10)
rospy.loginfo("aaa")
while not rospy.is_shutdown():
    robot_pose = tf2_geometry_msgs.PoseStamped()
    robot_pose.header.frame_id = "base_link"
    robot_pose.header.stamp = rospy.Time(0)
    robot_pose.pose.orientation.w = 1.0
    try:
        global_pose = tfBuffer.transform(robot_pose,"odom")
        # global_pose = listener.lookupTransform("base_link","map")
    except:
        rospy.loginfo("WARNING: tf map to base_link not found.")
        rospy.sleep(1)
        continue

    quaternion_rotation = [global_pose.pose.orientation.x,global_pose.pose.orientation.y,global_pose.pose.orientation.z,global_pose.pose.orientation.w]
    euler_rotation = T.euler_from_quaternion(quaternion_rotation)
    yaw = euler_rotation[2]
    x = np.array([global_pose.pose.position.x,global_pose.pose.position.y,yaw])
    rospy.loginfo(f"x:{global_pose.pose.position.x:.1f},y:{global_pose.pose.position.y:.1f},yaw:{yaw:.1f}")
    # index_near = get_near_obs(x,obs,Nnear)
    obs_diff = np.apply_along_axis(lambda o:(o[0]-x[0])**2+(o[1]-x[1])**2,1,obs)
    index_ = np.argsort(obs_diff)
    index_near = index_[:Nnear]
    obs_nears = np.array(obs[index_near],dtype=np.float64)
    obs_nears_vel = np.array(obs_vel[index_near],dtype=np.float64)
    vel_dot = np.apply_along_axis(lambda o:np.sqrt(o[0]**2+o[1]**2),1,obs_vel)
    obs_rob_rad = np.apply_along_axis(lambda o:np.arctan2(x[1]-o[1],x[0]-o[0]),1,obs)
    vel_theta = np.apply_along_axis(lambda o:np.arctan2(o[1],o[0]),1,obs_vel)
    vel_von_fnc = np.frompyfunc(lambda d,t,r:50*vonmises.pdf(t-r,d) if d != 0 else 1.,3,1)
    vel_von = vel_von_fnc(vel_dot,vel_theta,obs_rob_rad)
    # rospy.loginfo(obs_vel)
    params = np.concatenate([x.flatten(),x_ref.flatten(),obs.flatten(),obs_vel.flatten(),obs_nears.flatten(),obs_nears_vel.flatten(),vel_von.flatten()])
    msg.parameter = params
    pub.publish(msg)


    rate.sleep()