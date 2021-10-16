#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import os
import cv2
import numpy as np


class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(2)
        
        # Subscribers
        sub = rospy.Subscriber('/xtion/rgb/image_color', Image, self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)


if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Nodo()

