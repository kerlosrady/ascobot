#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String
import sys


rospy.init_node("mobile_node")
movement_publisher= rospy.Publisher('/mobile_base_controller/cmd_vel', Twist , queue_size=10)
stop_publisher= rospy.Publisher('base_state', String , queue_size=10)

def callback(data):
    if data.data==5:
    	for_ctrl()
    if data.data==4:
    	global vx
        vx.data=2
        print("rotate!!!")
        for_ctrl()

vx=Float32()
vx.data=0

movement_cmd2 = Twist()  
movement_cmd2.linear.x = 0
movement_cmd2.linear.y = 0
movement_cmd2.linear.z = 0
movement_cmd2.angular.x = 0
movement_cmd2.angular.y = 0              
movement_cmd2.angular.z = 0

def for_ctrl():
    rate = rospy.Rate(3) # 10hz
    movement_cmd = Twist()
    
    movement_cmd.linear.x = 1.2
    movement_cmd.linear.y = 0
    movement_cmd.linear.z = 0
    movement_cmd.angular.x = 0
    movement_cmd.angular.y = 0
    movement_cmd.angular.z = -0.03       
    
    def stop_callback(data):
        if data.data==15.0:
            #print("kkk")
            global vx
            vx.data=1
            return
        if data.data==666:
            global vx
            vx.data=3
            return
    rospy.Subscriber("lidar_reading", Float32, stop_callback)
           
    while not rospy.is_shutdown():
        if vx.data==0:
            movement_publisher.publish(movement_cmd)
        if vx.data==1:
            movement_publisher.publish(movement_cmd2)
            stop_publisher.publish("arrived")
            baseNode()
        if vx.data==2:
            movement_publisher.publish(movement_cmd3)
            stop_publisher.publish("rotating") 
        if vx.data==3:
            movement_publisher.publish(movement_cmd2)
            stop_publisher.publish("rotated")
            baseNode()
        rate.sleep()

movement_cmd3 = Twist()
movement_cmd3.linear.x = 0
movement_cmd3.linear.y = 0
movement_cmd3.linear.z = 0
movement_cmd3.angular.x = 0
movement_cmd3.angular.y = 0              
movement_cmd3.angular.z = 0.3
    
def baseNode():
      rospy.Subscriber('chatter_1', Float32, callback)
      print("waiting for 4 or 5")
      sys.setrecursionlimit(10000): 
      rospy.spin()
            
if __name__=='__main__':
     try:
        baseNode()
     except rospy.ROSInterruptException:
       pass
