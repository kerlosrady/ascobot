#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys

rospy.init_node("mobile_node")
movement_publisher= rospy.Publisher('/mobile_base_controller/cmd_vel', Twist , queue_size=10)

def for_ctrl():
    rate = rospy.Rate(10) # 10hz
    movement_cmd = Twist()
    movement_cmd.linear.x = 1.5
    movement_cmd.linear.y = 0
    movement_cmd.linear.z = 0
    movement_cmd.angular.x = 0
    movement_cmd.angular.y = 0              
    movement_cmd.angular.z = 0
    def stop_callback(event):
        rospy.signal_shutdown("Just stopping publishing...")

    rospy.Timer(rospy.Duration(2), stop_callback)

    while not rospy.is_shutdown():
        movement_publisher.publish(movement_cmd)
        rate.sleep()

def rot_ctrl():
    rate = rospy.Rate(10) # 10hz
    movement_cmd = Twist()
    movement_cmd.linear.x = 0
    movement_cmd.linear.y = 0
    movement_cmd.linear.z = 0
    movement_cmd.angular.x = 0
    movement_cmd.angular.y = 0              
    movement_cmd.angular.z = 0.8
    def stop_callback(event):
        rospy.signal_shutdown("Just stopping publishing...")

    rospy.Timer(rospy.Duration(1), stop_callback)

    while not rospy.is_shutdown():
        movement_publisher.publish(movement_cmd)
        rate.sleep()

            
if __name__=='__main__':
     try:
        for_ctrl()
        rot_ctrl()
     except rospy.ROSInterruptException:
       pass
