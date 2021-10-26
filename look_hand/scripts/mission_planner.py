#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32,Float64,Float32MultiArray
from nav_msgs.msg import Path
from math import fabs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import tf2_ros
import tf
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, WrenchStamped, TransformStamped
import geometry_msgs.msg 



class TransformServices():
    def __init__(self):
        self.transformer_listener = tf.TransformListener()
        self.transformer_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def transform_poses(self, target_frame, source_frame, pose_arr):
        """
        Transform poses from source_frame to target_frame
        """
        trans_pose_arr = PoseArray()
        for i in range(len(pose_arr.poses)):
            trans_pose = PoseStamped()
            pose = PoseStamped()
            pose.header.frame_id = source_frame
            pose.pose = pose_arr.poses[i]
            self.transformer_listener.waitForTransform(
                target_frame, source_frame, rospy.Time(), rospy.Duration(1))
            trans_pose = self.transformer_listener.transformPose(
                target_frame, pose)
            trans_pose_arr.poses.append(trans_pose.pose)

        trans_pose_arr.header.frame_id = target_frame
        trans_pose_arr.header.stamp = rospy.Time()
        return trans_pose_arr

    def lookup_transform(self, target_frame, source_frame):
        self.transformer_listener.waitForTransform(
            target_frame, source_frame, rospy.Time(), rospy.Duration(1))
        t, r = self.transformer_listener.lookupTransform(
            target_frame, source_frame, rospy.Time())
        pose = Pose()
        pose.position.x = t[0]
        pose.position.y = t[1]
        pose.position.z = t[2]
        pose.orientation.x = r[0]
        pose.orientation.y = r[1]
        pose.orientation.z = r[2]
        pose.orientation.w = r[3]
        return pose

    def create_frame(self, ref_frame, moving_frame, new_frame):
        pose = self.lookup_transform(ref_frame, moving_frame)
        return self.create_frame_at_pose(pose, ref_frame, new_frame)
    
    def create_frame_at_pose(self, pose, ref_frame, new_frame):
        translation = [pose.position.x, pose.position.y, pose.position.z]
        orientation = [pose.orientation.x, pose.orientation.y,
                       pose.orientation.z, pose.orientation.w]
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = ref_frame
        static_transformStamped.child_frame_id = new_frame

        static_transformStamped.transform.translation.x = translation[0]
        static_transformStamped.transform.translation.y = translation[1]
        static_transformStamped.transform.translation.z = translation[2]

        static_transformStamped.transform.rotation.x = orientation[0]
        static_transformStamped.transform.rotation.y = orientation[1]
        static_transformStamped.transform.rotation.z = orientation[2]
        static_transformStamped.transform.rotation.w = orientation[3]
        self.transformer_broadcaster.sendTransform(static_transformStamped)
        return pose





class mission_planner():

	def __init__(self):


		self.state=1
		self.sub1=rospy.Subscriber("control_arm", Float32, self.control_arm_callback)
		self.sub2=rospy.Subscriber("grip",Float32, self.grip_callback)
		self.sub3=rospy.Subscriber("/cansPos",Path, self.can_detection_callback)
		#self.msgcamera=Path()
		#self.tf = TransformListener()
		#rospy.Subscriber("control_base", base_states, control_base_callback)
		self.pubr = rospy.Publisher('rarm',Float32MultiArray, queue_size=10)
		self.publ = rospy.Publisher('larm',Float32MultiArray, queue_size=10)
		self.pub2= rospy.Publisher('gripper', Float32, queue_size=10)
		self.pub3= rospy.Publisher('chatter_1',Float32, queue_size=10)


		#self.pub4= rospy.Publisher('/cansPos', Float32, queue_size=10)
		#self.pub5= rospy.Publisher('can_detection', Float32, queue_size=10)

		self.cans_detected = False
		rate = rospy.Rate(1) # 10hz
		self.done= False
		self.cycle=0
	
		self.totalCans=12

		while not rospy.is_shutdown():

			# if state==0 and table_depth >= threshold:
			#         #publish certain action to get back
			#         state=1
			#         msgb= base_data()
			#         msgb.loc= heading_pos
			#         pub1.publish(msgb)g

			#if state==1 and table_depth - threshold >=0.1:
			if self.state==1:
				#msgb = base_data()
				#msgb.stop =True
				#pub1.publish(msgb)
				print(self.state)

				
				rospy.sleep(1)
				self.forward= 5.0
				self.pub3.publish(self.forward)
				rospy.sleep(5.0) 
				self.state =2
				
			
			if self.state==2:
				print(self.state)
				self.pub3.publish(6)
				self.state=3
				


			if self.state==3 and self.cans_detected is True:
				self.state = 4
				self.execute_state=1
				self.cycle= self.cycle+1

				# if execute_state==1:
				while (self.done is not True):

							
					if self.execute_state ==1 :
						

						#arm 1
						pose_goal1= Float32MultiArray.data
						pose_goal1[0]=self.finalPoints.poses[0].pose.position.x
						pose_goal1[1]=self.finalPoints.poses[0].pose.position.y
						pose_goal1[2]=self.finalPoints.poses[0].pose.position.z
						pose_goal1[3]=0
						pose_goal1[4]=0
						pose_goal1[5]=0
						pose_goal1[6]=1
						pose_goal1[0]=1

						#pose_goal1 = geometry_msgs.msg.Pose()
					    	#pose_goal1.orientation.w =1
					    	#pose_goal1.position.x = point1Tr.point.x
					    	#pose_goal1.position.y = point1Tr.point.y
					    	#pose_goal1.position.z = point1Tr.point.z 
					    	#pose_goal1.orientation.x =0
					    	#pose_goal1.orientation.y =0
					    	#pose_goal1.orientation.z =1

						self.pubr.publish(pose_goal1)
						

						#arm 2
						

						pose_goal2= Float32MultiArray.data
						pose_goal2[0]=self.finalPoints.poses[1].pose.position.z
						pose_goal2[1]=self.finalPoints.poses[1].pose.position.z
						pose_goal2[2]=self.finalPoints.poses[1].pose.position.z
						pose_goal2[3]=0
						pose_goal2[4]=0
						pose_goal2[5]=0
						pose_goal2[6]=1
						pose_goal2[0]=1
						
						self.publ.publish(pose_goal2)
						self.execute_state = 2
		
					if self.execute_state == 2 and self.reach_target == True:
						self.execute_state = 3
						self.reach_target= False
						self.pub2.publish(True)


					if self.execute_state == 2 and grip_target == True:
							
						z1= z1+0.1
						z2= z2=0.1
						msg1 = Float32MultiArray
						msg1 = [x1, y1, z1,x2,y2,z2]
						self.execute_state = 3
						grip_target= False
						self.done= True



	def control_arm_callback(self,data):

		if data.data is True:
			self.reach_target= True

	def can_detection_callback(self,data):
		#can1_posx= data.data[0]
		#can1_posy= data.data[1]
		#can1_posz= data.data[2]

		#can2_posx= data.data[3]
		#can2_posy= data.data[4]
		#can2_posz= data.data[5]
		#print("cans detected")
		if self.state==3:

			self.cans_detected= True
			print(self.cans_detected)
			self.msgcamera_id= data.header.frame_id
			self.msgcamera_poses =data.poses
			
			num_cans= len(self.msgcamera_poses)
			campos= np.(num_cans,3)
			for i in range(len(num_cans)):
				tempar= np.ones(3)
				tempar[0]= self.msgcamera_poses[i].pose.position.x
				tempar[1]= self.msgcamera_poses[i].pose.position.y
				tempar[2]= self.msgcamera_poses[i].pose.position.z
				campos[i]=tempar

			col_y=campos[campos[:,1].argsort()]
			
			if num_cans%4== 0:

				first_row= col_y[-4:-1]
				col_x = first_row[first_row[:,0].argsort()]
				selectedCans =np.ones(2,3)
				selectedCans[0]= col_x[0]
				selectedCans[1]= col_x[-1]

			else:
				first_row= col_y[:1]
				selectedCans= first_row
				
			tfs= PoseArray()
			tfs.header.frame_id= self.msgcamera_id

			tfsp1 = PoseStamped()
			tfsp1.header.frame_id = self.msgcamera_id
			tfsp1.pose.position.x= selectedCans[0][0]
			tfsp1.pose.position.y= selectedCans[0][1]
			tfsp1.pose.position.z= selectedCans[0][2]
			tfs.poses.append(tfsp1)

			tfsp2 = PoseStamped()
			tfsp2.header.frame_id = self.msgcamera_id
			tfsp2.pose.position.x= selectedCans[0][0]
			tfsp2.pose.position.y= selectedCans[0][1]
			tfsp2.pose.position.z= selectedCans[0][2]
			tfs.poses.append(tfsp2)
			
			Trans=TransformServices()
			self.finalPoints = Trans.transform_poses(self.msgcamera_id,'/base_link',tfs)
			print(selectedCans)
			
			print(data.poses)
		
		
		
		
		
		
            	#print position, quaternion
		
		


	def grip_callback(self,data):
		if data.data is True:
			grip_target= True


if __name__=='__main__':

	rospy.init_node("mission_planner")

	
	try:
     		mission_planner()
     	except rospy.ROSInterruptException:
     		pass


	






	


