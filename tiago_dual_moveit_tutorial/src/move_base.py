#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import sys


rospy.init_node("mobile_node")
movement_publisher= rospy.Publisher('/mobile_base_controller/cmd_vel', Twist , queue_size=10)

def callback(data):
    if data.data==5:
    	for_ctrl()
    if data.data=='4':
    	rot_ctrl()
    	
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
    movement_cmd.angular.z = 0.3
    def stop_callback(event):
        rospy.signal_shutdown("Just stopping publishing...")

    rospy.Timer(rospy.Duration(3), stop_callback)

    while not rospy.is_shutdown():
        movement_publisher.publish(movement_cmd)
        rate.sleep()

def baseNode():
      rospy.Subscriber('chatter_1', Float32, callback)
      rospy.spin()
            
if __name__=='__main__':
     try:
        baseNode()
     except rospy.ROSInterruptException:
       pass
