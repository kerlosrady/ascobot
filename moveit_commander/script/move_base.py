#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String
import sys

def init():
	rospy.init_node("mobile_node")
	movement_publisher= rospy.Publisher('/mobile_base_controller/cmd_vel', Twist , queue_size=10)
	stop_publisher= rospy.Publisher('base_state', String , queue_size=10)

def callback(data):
    if data.data==5:
    	for_ctrl()

    if data.data==4:
    	rot_ctrl()
    	
def callback2(data):
    if data.data==999:
    	stop_publisher.publish("arrived") 
    	stop_callback()
    if data.data==666:
    	stop_publisher.publish("rotated") 
    	stop_callback()
    	
def for_ctrl():
    rate = rospy.Rate(10) # 10hz
    movement_cmd = Twist()
    movement_cmd.linear.x = 0.8
    movement_cmd.linear.y = 0
    movement_cmd.linear.z = 0
    movement_cmd.angular.x = 0
    movement_cmd.angular.y = 0              
    movement_cmd.angular.z = 0

    while not rospy.is_shutdown():
        movement_publisher.publish(movement_cmd)
        rate.sleep()
        
def stop_callback():
        rospy.signal_shutdown("Just stopping publishing...")
        init()
        
def rot_ctrl():
    rate = rospy.Rate(10) # 10hz
    movement_cmd = Twist()
    movement_cmd.linear.x = 0
    movement_cmd.linear.y = 0
    movement_cmd.linear.z = 0
    movement_cmd.angular.x = 0
    movement_cmd.angular.y = 0              
    movement_cmd.angular.z = 0.26
    while not rospy.is_shutdown():
        movement_publisher.publish(movement_cmd)
        rate.sleep()

def baseNode():
      rospy.Subscriber('chatter_1', Float32, callback)
      rospy.Subscriber('Lidar_reading', Float32, callback2)
      rospy.spin()
            
if __name__=='__main__':
     try:
        init()
        baseNode()
     except rospy.ROSInterruptException:
       pass
