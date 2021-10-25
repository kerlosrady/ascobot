#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

# initialize ros node
rospy.init_node("publisher_node")

# create publisher
publisher = rospy.Publisher("/phrase", Float32MultiArray, queue_size = 10)

while not rospy.is_shutdown():
	msg = Float32MultiArray()
	msg.data = [0.69375,-0.13761,0.7151]
	
	publisher.publish(msg)
	rospy.sleep(1)
