#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs import msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String


class head:
    joint_names = ["head_1_joint", "head_2_joint"]
    tucked  = [0,-0.6]

    def __init__(self):
        rospy.loginfo("Waiting for arm_controller...")
        self.client = actionlib.SimpleActionClient("/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.client.wait_for_server()
        rospy.loginfo("...connected.")

        self.state_recv = False
        self.sub = rospy.Subscriber("joint_states", JointState, self.state_callback)

    def state_callback(self, msg):
        self.state_recv = True

    def head(self,x,y):
        while not self.state_recv:
            rospy.loginfo("Waiting for controllers to be up...")
            rospy.sleep(0.1)

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = [x,y]
        trajectory.points[0].velocities = [0.0 for i in self.joint_names]
        trajectory.points[0].accelerations = [0.0 for i in self.joint_names]
        trajectory.points[0].time_from_start = rospy.Duration(1.0)

        rospy.loginfo("Tucking head...")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(3.0))
        rospy.loginfo("...done")

def callback(data):
    msg = Float32()
    msg = 1.0
    publisher = rospy.Publisher("/ak_head", Float32, queue_size = 10)

    if data.data == 6 :
        t = head()
        t.head(0,-0.7)
        publisher.publish(msg)

    if data.data == 66 :
        t.head(0,0)
        publisher.publish(msg)

    if data.data==777:
        t = head()
        # t.head(1,-0.7)          #Done by Hend
        t.head(0,-0.7)
        publisher.publish(2)
		
if __name__ == "__main__":
    rospy.init_node("tuck_my_arm")
    rospy.Subscriber('chatter_1', Float32, callback)     
    rospy.spin()
    
