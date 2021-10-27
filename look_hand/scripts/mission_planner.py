#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32,Float64,Float32MultiArray, String
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
		self.subra=rospy.Subscriber("confirmation_rh", String, self.Rcontrol_arm_callback)
		self.subla=rospy.Subscriber("confirmation_lh", String, self.Lcontrol_arm_callback)
		
		self.subgr=rospy.Subscriber("confirmation_gr",String, self.Rgrip_callback)
		self.subgl=rospy.Subscriber("confirmation_gl",String, self.Lgrip_callback)
		
		self.subB=rospy.Subscriber("base_state",String, self.base_callback)
		
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

		self.done= False
		self.LgripState=False
		self.RgripState=False
		self.LreleaseState=False
		self.RreleaseState=False
		self.RarmReach=False
		self.LarmReach=False
		self.BarrivalState=False
		self.BrotateState= False
		
		rate = rospy.Rate(1) # 10hz
		
		self.cycle=0
	
		self.totalCans=12
		self.finalPoints= PoseArray()

		while not rospy.is_shutdown():

			# if state==0 and table_depth >= threshold:
			#         #publish certain action to get back
			#         state=1
			#         msgb= base_data()
			#         msgb.loc= heading_pos
			#         pub1.publish(msgb)g

			#if state==1 and table_depth - threshold >=0.1:
			print("s", self.state , self.BarrivalState ,self.cans_detected)
			if self.state==1:
		
				print(self.state , self.BarrivalState ,self.cans_detected)

				self.pub3.publish(5)
				self.state =2
				
			
			if self.state==2 and self.BarrivalState==True:
				print(self.state , self.BarrivalState ,self.cans_detected)
				self.pub3.publish(6)
				self.state=3
				


			if self.state==3 and self.cans_detected == True:
				print(self.state , self.BarrivalState ,self.cans_detected)
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
		
					if self.execute_state == 2 and self.LarmReach==True and self.RarmReach==True:
						self.execute_state = 3
						self.LarmReach=False
						self.LarmReach=False
						self.pub2.publish(11)


					if self.execute_state == 2 and  self.LgripState== True and self.RgripState== True:
							
						pose_goal1= Float32MultiArray.data
						pose_goal1[0]=self.finalPoints.poses[0].pose.position.x
						pose_goal1[1]=self.finalPoints.poses[0].pose.position.y
						pose_goal1[2]=self.finalPoints.poses[0].pose.position.z+0.1
						pose_goal1[3]=0
						pose_goal1[4]=0
						pose_goal1[5]=0
						pose_goal1[6]=1
						pose_goal1[0]=1

						self.pubr.publish(pose_goal1)
						

						#arm 2
						
						pose_goal2= Float32MultiArray.data
						pose_goal2[0]=self.finalPoints.poses[1].pose.position.z
						pose_goal2[1]=self.finalPoints.poses[1].pose.position.z
						pose_goal2[2]=self.finalPoints.poses[1].pose.position.z+0.1
						pose_goal2[3]=0
						pose_goal2[4]=0
						pose_goal2[5]=0
						pose_goal2[6]=1
						pose_goal2[0]=1
						
						self.publ.publish(pose_goal2)

						self.publ.publish(pose_goal2)
						self.done= True
						self.execute_state = 3

					if self.execute_state==3 and self.RarmReach==True and self.LarmReach==True:
						self.pub3.publish(4)
						self.RarmReach= False
						self.LarmReach= False
						self.execute_state=4
					if self.execute_state==4 and self.BrotateState== True:
						self.BrotateState=False
						self.execute_state=5

					
			
			
	def base_callback(self,data):
		if data.data== "arrived":
			self.BarrivalState=True

		if data.data== "rotated":
			self.BrotateState=True
			

	def Rcontrol_arm_callback(self,data):
		if data.data== "rarm_done":
			self.RarmReach=True
	
	def Lcontrol_arm_callback(self,data):
		if data.data == "larm_done":
			self.LarmReach=True

	def Rgrip_callback(self,data):

		if data.data == "gripped":
			
			self.LgripState= True

		if date.data =="released":
			self.LreleaseState= True

	def Lgrip_callback(self,data):
		if data.data == "gripped":
			self.LgripState= True

		if data.data =="released":
			
			self.LreleaseState= True


	def can_detection_callback(self,data):
		#can1_posx= data.data[0]
		#can1_posy= data.data[1]
		#can1_posz= data.data[2]

		#can2_posx= data.data[3]
		#can2_posy= data.data[4]
		#can2_posz= data.data[5]
		#print("cans detected")
		if self.state==3 and self.cans_detected ==False:

			
			#print(self.cans_detected)
			self.msgcamera_id= data.header.frame_id
			self.msgcamera_poses =data.poses
			
			num_cans= len(self.msgcamera_poses)
			campos= np.ones((num_cans,3))
			for i in range(num_cans):
				tempar= np.ones(3)
				tempar[0]= self.msgcamera_poses[i].pose.position.x
				tempar[1]= self.msgcamera_poses[i].pose.position.y
				tempar[2]= self.msgcamera_poses[i].pose.position.z
				campos[i]=tempar
			print(campos)
			#col_y=campos[i in campos[:,1].argsort()]
			col_y=campos[np.argsort(campos[:,1])]
			print("col_y",col_y)
			#col_y = col_y[0]
			
			if num_cans%4== 0:

				first_row= col_y[-4:]
				print("1st row", first_row)
				#fri = first_row[:,0].argsort()
				#print(fri)
				#col_x = first_row[j in fri]
				col_x=first_row[np.argsort(first_row[:,0])]
				#col_x = col_x[0]
				print("colx",col_x)
				selectedCans =np.ones((2,3))
				selectedCans[0]= col_x[0]
				selectedCans[1]= col_x[-1]
				print("selected cans",selectedCans)

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
			self.cans_detected= True
		
		
		
		
		
            	#print position, quaternion
		


if __name__=='__main__':

	rospy.init_node("mission_planner")

	
	try:
     		mission_planner()
     	except rospy.ROSInterruptException:
     		pass


	






	


