#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String
import sys


rospy.init_node("mobile_node")
movement_publisher= rospy.Publisher('/mobile_base_controller/cmd_vel', Twist , queue_size=10)
stop_publisher= rospy.Publisher('base_state', String , queue_size=10)


vx=Float32()
vx.data=0

movement_cmd2 = Twist()  
movement_cmd2.linear.x = 0
movement_cmd2.linear.y = 0
movement_cmd2.linear.z = 0
movement_cmd2.angular.x = 0
movement_cmd2.angular.y = 0              
movement_cmd2.angular.z = 0

movement_cmdb = Twist()  
movement_cmdb.linear.x = -0.2
movement_cmdb.linear.y = 0
movement_cmdb.linear.z = 0
movement_cmdb.angular.x = 0
movement_cmdb.angular.y = 0              
movement_cmdb.angular.z = 0

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
        if data.data==20.0:
            #print("kkk")
            global vx
            #vx.data=5
            return
        if data.data==666:
            global vx
            vx.data=3
            return
        if data.data==66666:
            global vx
            vx.data=4
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
            movement_publisher.publish(movement_cmdb)
            stop_publisher.publish("going_back")
        if vx.data==4:
            movement_publisher.publish(movement_cmd2)
            stop_publisher.publish("rotated")
            #vx.data=1 
            baseNode()
        if vx.data==5:
            movement_publisher.publish(movement_cmd5)
            stop_publisher.publish("arrived") 
            baseNode()
        rate.sleep()

movement_cmd3 = Twist()
movement_cmd3.linear.x = 0
movement_cmd3.linear.y = 0
movement_cmd3.linear.z = 0
movement_cmd3.angular.x = 0
movement_cmd3.angular.y = 0              
movement_cmd3.angular.z = 0.3
    
movement_cmd5 = Twist()
movement_cmd5.linear.x = 0.1
movement_cmd5.linear.y = 0
movement_cmd5.linear.z = 0
movement_cmd5.angular.x = 0
movement_cmd5.angular.y = 0              
movement_cmd5.angular.z = -0.15

def callback(data):
    print ("I am sub")
    if data.data==5:
    	for_ctrl()
    if data.data==4:
    	global vx
        vx.data=2
        print("rotate!!!")
        for_ctrl()
    if data.data==8:
        vx.data=5        
        print("back to table!!!")
        for_ctrl()

def baseNode():
      rospy.Subscriber('chatter_1', Float32, callback)
      print("waiting for 4 or 5")
      sys.setrecursionlimit(10000)
      rospy.spin()
            
if __name__=='__main__':
     try:
        baseNode()
     except rospy.ROSInterruptException:
       pass
