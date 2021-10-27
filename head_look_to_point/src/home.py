#!/usr/bin/env python

import rospy # Python library for ROS
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32


def callback(msg):

    for I in range(0,360):
	if I == 180 :
	    print(msg.ranges[I])
        if msg.ranges[I] < 0.8 :
            print("You should Stop")
            pub2.publish(999)
        else:
            pass		
	
rospy.init_node('check_obstacle') # Initializes a node      
# Subscriber object which will listen "LaserScan" type messages
sub = rospy.Subscriber("/scan", LaserScan, callback)  
print(type(sub))
# outgoing message queue used for asynchronous publishing
pub2 = rospy.Publisher("/lidar_reading", Float32, queue_size=10)
for I in range(0,361):
	if I == 360 :
	    print(sub.ranges[I])
rospy.spin() # Loops infinitely until someone stops the program execution
