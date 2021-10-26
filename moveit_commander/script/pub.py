#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

# initialize ros node
rospy.init_node("publisher_node")

# create publisher
<<<<<<< HEAD
publisher = rospy.Publisher('chatter_2', Float32MultiArray, queue_size = 10)
=======
publisher = rospy.Publisher('larm', Float32MultiArray, queue_size = 10)
>>>>>>> cdc496d413f1c3b4769e84c3ed25489e1e82d748
publisher2 = rospy.Publisher('gripper', Float32, queue_size = 10)

while not rospy.is_shutdown():
	msg = Float32MultiArray()
<<<<<<< HEAD
	msg.data = [0.6937500,-0.1376100,0.7151]
        publisher.publish(11)
=======
	msg.data = [0.064765,0.83785,0.7151]
        #publisher2.publish(11)
>>>>>>> cdc496d413f1c3b4769e84c3ed25489e1e82d748
	publisher.publish(msg)
	rospy.sleep(1)
