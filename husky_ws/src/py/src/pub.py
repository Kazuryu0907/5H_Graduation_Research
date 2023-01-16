#!/usr/bin/env python3
from subprocess import call
import rospy
from gazebo_msgs.msg import ModelStates,ModelState
from obstacle_detector.msg import Obstacles,CircleObstacle

rospy.init_node("publisher")

states = [ModelState() for _ in range(3)]
_vels = [CircleObstacle() for _ in range(3)]
pub_indexs = []
sub_indexs = []
vels = [CircleObstacle() for _ in range(3)]
#target->moving model_name 
#index-> subscribe model_index
def callback(msg:ModelStates,pub_targets,sub_targets):
    #only run one time
    if not hasattr(callback,"flag"):
        callback.flag = True
        for t in pub_targets:
            pub_indexs.append(msg.name.index(t))
        for t in sub_targets:
            sub_indexs.append(msg.name.index(t))

    for i in range(len(pub_indexs)):
        states[i].model_name = msg.name[pub_indexs[i]]
        states[i].pose.position.x = msg.pose[sub_indexs[i]].position.x
        states[i].pose.position.y = msg.pose[sub_indexs[i]].position.y
        states[i].pose.position.z = 0
        _vels[i].center.x = msg.pose[sub_indexs[i]].position.x
        _vels[i].center.y = msg.pose[sub_indexs[i]].position.y

def obs_callback(event):
    for i in range(3):
        vels[i].velocity.x = 0.1*(_vels[i].center.x-vels[i].center.x)
        vels[i].velocity.y = 0.1*(_vels[i].center.y-vels[i].center.y)
        vels[i].center.x = _vels[i].center.x
        vels[i].center.y = _vels[i].center.y

rospy.Timer(rospy.Duration(1./10),obs_callback)


# sub = rospy.Subscriber("/gazebo/model_states",ModelStates,lambda x : callback(x,["human_collision","human_collision2","human_collision3"],["actor","actor2","actor3"]))
sub = rospy.Subscriber("/gazebo/model_states",ModelStates,lambda x : callback(x,["human_collision2","human_collision3"],["actor2","actor3"]))
pub_obs = rospy.Publisher("~/obs",Obstacles,queue_size=10)
# sub = rospy.Subscriber("/gazebo/model_states",ModelStates,lambda x : callback(x,["human_collision1","human_collision2"],["actor1","actor2"]))

pub = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=10)
rate = rospy.Rate(50)
obs = Obstacles()
while not rospy.is_shutdown():

    pub.publish(states[0])
    pub.publish(states[1])
    # pub.publish(states[2])
    obs.circles = vels
    pub_obs.publish(obs)
    rate.sleep()
