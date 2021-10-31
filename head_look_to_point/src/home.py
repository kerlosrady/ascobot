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
		print(msg.ranges[I])
        if msg.ranges[I] < 1.3 :
            print("You should Stop bcuz of for")
            if n.data==0:
                pub2.publish(15.0)
                n.data=1
            # print("Stop")
            return
        else:
            pass

	# print(msg.ranges[333])

        if msg.ranges[200] < 0.4:
            print("You should Stop bcuz of 200")
            if n.data<5:
                pub2.publish(666)
                n.data=n.data+1	
            return
        
        if msg.ranges[360] > 0.45 and msg.ranges[360] < 0.47:
            print("You should Stop bcuz of multi")
            if n.data<8:
                pub2.publish(66666)
                n.data=n.data+1	
            return
        
rospy.init_node('check_obstacle') # Initializes a node      
# Subscriber object which will listen "LaserScan" type messages
sub = rospy.Subscriber("/scan", LaserScan, callback)  

# outgoing message queue used for asynchronous publishing

rospy.spin() # Loops infinitely until someone stops the program execution
