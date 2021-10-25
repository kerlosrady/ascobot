#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

# initialize ros node
rospy.init_node("publisher_node")

# create publisher
publisher = rospy.Publisher("chatter_2", Float32MultiArray, queue_size = 10)

while not rospy.is_shutdown():
	msg = Float32MultiArray()
	msg.data = [0.472310,-0.3871300,0.5708900]
	
	publisher.publish(msg)
	rospy.sleep(1)
