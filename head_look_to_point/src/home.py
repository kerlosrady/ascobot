#!/usr/bin/env python

import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32

pub2 = rospy.Publisher("lidar_reading", Float32, queue_size=10)
n=Float32()
n.data=0
def callback(msg):
    global n
    for I in range(0,360):
        if msg.ranges[I] < 1.3 :
            # print("You should Stop")
            if n.data==0:
                pub2.publish(15.0)
                n.data=1
            # print("Stop")
            return

        else:
            pass
        if msg.ranges[360] < 0.9 :
            print("You should Stop")
	    pub2.publish(15.0)
            if n.data==1:
                pub2.publish(666)
                n.data=2		
	
rospy.init_node('check_obstacle') # Initializes a node      
# Subscriber object which will listen "LaserScan" type messages
sub = rospy.Subscriber("/scan", LaserScan, callback)  

# outgoing message queue used for asynchronous publishing

rospy.spin() # Loops infinitely until someone stops the program execution
