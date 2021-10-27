#!/usr/bin/env python

import rospy # Python library for ROS

from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32

#ranges array contains alot of messages, each message is a laser beam 
#laser beams cover 180 deg
#ranges are inf(no obstacles on the right side),values(at the center)(represent center distance to an obstacle right in front of the robot),inf(no obstacles on the left side)



def callback(msg):

    for I in range(0,360):
        if msg.ranges[I] < 0.8 :
            print("You should Stop")
            pub2.publish(999)
        else:
            pass
         #any(msg.ranges[100:360]<0.5): #when the center distance to the obstacle becomes less than 0.5 the robot should stop
		

	
rospy.init_node('check_obstacle') # Initializes a node      
    
sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages
                                                      # from the "/scan" Topic and call the "callback" function
						      # each time it reads something from the Topic

#pub = rospy.Publisher("/dis_action", String::std, queue_size=10)  # Publisher object which will publish "Twist" type messages
                            				 # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                         # outgoing message queue used for asynchronous publishing
pub2=rospy.Publisher("Lidar_reading", Float32, queue_size=10)

rospy.spin() # Loops infinitely until someone stops the program execution

