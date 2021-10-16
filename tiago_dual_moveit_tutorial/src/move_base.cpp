import rospy
from geometry_msgs.msg import Twist
import sys

rospy.init_node("mobile_node")
movement_publisher= rospy.Publisher('cmd_vel', Twist , queue_size=10)
movement_cmd = Twist()
movement_cmd.linear.x = 0.2
movement_cmd.linear.y = 0
movement_cmd.linear.z = 0
movement_cmd.angular.x = 0
movement_cmd.angular.y = 0              
movement_cmd.angular.z = 0
movement_publisher.publish(movement_cmd)




rospy.spin()      
 
