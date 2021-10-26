#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

# initialize ros node
rospy.init_node("publisher_node")

# create publisher
publisher = rospy.Publisher('chatter_2', Float32MultiArray, queue_size = 10)
publisher2 = rospy.Publisher('gripper', Float32, queue_size = 10)

while not rospy.is_shutdown():
	msg = Float32MultiArray()
	msg.data = [0.6937500,-0.1376100,0.7151]
        publisher.publish(11)
	publisher.publish(msg)
	rospy.sleep(1)
