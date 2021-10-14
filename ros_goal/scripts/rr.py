#!/usr/bin/env python3

import rospy
from moveit_commander import MoveGroupCommander
from sensor_msgs.msg import JointState

rospy.init_node('robot_controller_movveit')

group = MoveGroupCommander('both_arms_torso')

msg = JointState()

msg.name = ['arm_right_4_joint' , 'arm_joint_5_joint']
msg.position = [1 , 1] 

group.set_joint_value_target(msg)
group.go()
