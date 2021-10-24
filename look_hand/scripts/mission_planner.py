#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Float32

#from look_hand.srv import shelf_detection

state=1
rospy.init_node('mission_planner')


#rospy.init_node('mission_planning')
#rospy.Subscriber("detection", object_pos, vision_callback)
#rospy.Subscriber("grip", bool, grip_callback)

def mission_planner():

	rospy.Subscriber("control_arm", Int32, control_arm_callback)
	rospy.Subscriber("grip",Int32, grip_callback)
	rospy.Subscriber("can_detected",Float32MultiArray, can_detection_callback)


	#rospy.Subscriber("control_base", base_states, control_base_callback)
        self.pub = rospy.Publisher('arm_actions',Float32MultiArray, queue_size=10)
        self.pub2= rospy.Publisher('gripper', Int32, queue_size=10)
	self.pub3= rospy.Publisher('chatter_1',Float32, queue_size=10)

        self.pub3= rospy.Publisher('camera_pos', Int32, queue_size=10)
	self.pub4= rospy.Publisher('can_detection', Int32, queue_size=10)
        rate = rospy.Rate(10) # 10hz


	while not rospy.is_shutdown():

		# if state==0 and table_depth >= threshold:
		#         #publish certain action to get back
		#         state=1
		#         msgb= base_data()
		#         msgb.loc= heading_pos
		#         pub1.publish(msgb)g

		#if state==1 and table_depth - threshold >=0.1:
		if state==1:
			#msgb = base_data()
			#msgb.stop =True
			#pub1.publish(msgb)
			#state =2
			pub3.publish(5)
			

		if state==2 and cans_detected is True:
			state = 3
			execute_state=1
			cycle= cycle+1

			# if execute_state==1:
			while (done is not True):

				if execute_state == 1:
							
					msg1 = Float32MultiArray
					msg1= [x1, y1, z1, x2, y2, z2]
					
					pub.publish(msg1)

				if execute_state == 1 and reach_target == True:
					execute_state = 2
					reach_target= False
					pub2.publish(True)

					# rospy.wait_for_service('can_detection')

					# try:
					# 	can_pos = rospy.ServiceProxy('grip', grip)
					# 	resp1 = grip(1)
						
					# 	grip_target= resp1


					# except rospy.ServiceException as e:
					# 	print("Service call failed: %s" % e)
				

				if execute_state == 2 and grip_target == True:
						
					z1= z1+0.1
					z2= z2=0.1
					msg1 = Float32MultiArray
					msg1 = [x1, y1, z1,x2,y2,z2]
					execute_state = 3
					grip_target= False
					done= True


def control_arm_callback(data):

	if data.data is True:
		reach_target= True

def can_detection_callback(data):
	can1_posx= data.data[0]
	can1_posy= data.data[1]
	can1_posz= data.data[2]

	can2_posx= data.data[3]
	can2_posy= data.data[4]
	can2_posz= data.data[5]
	cans_detected= True



def grip_callback(data):
	if data.data is True:
		grip_target= True


if __name__=='__main__':

	rospy.init_node("mission_planner")

	
	try:
     		mission_planner()
     	except rospy.ROSInterruptException:
     		pass

	rospy.spin()
	






	


