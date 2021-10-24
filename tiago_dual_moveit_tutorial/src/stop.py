#!/usr/bin/env python
import rospy
from std_msgs.msg import Char

def firstNode(): 
    pub = rospy.Publisher('chatter_1', Char, queue_size=10)
    rospy.init_node('firstNode', anonymous=True)
    rate = rospy.Rate(0.1) # 0.5Hz once every 2 seconds

    while not rospy.is_shutdown():
        pub.publish('a')
        rate.sleep()
                      
if __name__=='__main__':
     try:
        firstNode()
     except rospy.ROSInterruptException:
       pass
