#! /usr/bin/env python
# -*- coding: utf-8 -*-

from numpy.core.numeric import ones
import rospy
from rospy.core import rospyinfo
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

    def transform_poses(self,source_frame,target_frame, pose_arr):
        """
        Transform poses from source_frame to target_frame
        """
        trans_pose_arr = PoseArray()
        for i in range(len(pose_arr.poses)):
            trans_pose = PoseStamped()
            pose = PoseStamped()
            pose.header.frame_id = source_frame
            pose.pose = pose_arr.poses[i]
            self.transformer_listener.waitForTransform(source_frame,target_frame, rospy.Time(), rospy.Duration(1))
            trans_pose = self.transformer_listener.transformPose( target_frame, pose)
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
 
		self.cans_detected = False
		self.shelf_detected=False
		self.done= False
		self.LgripState=False
		self.RgripState=False
		self.LreleaseState=False
		self.RreleaseState=False
		self.RarmReach=False
		self.LarmReach=False
		self.BarrivalState=False
		self.BrotateState= False
		self.execute_state= 0
		self.head_state=1
		rate = rospy.Rate(1) # 10hz
		
		self.cycle=0
	
		self.totalCans=12
		self.finalPoints= PoseArray()


		while not rospy.is_shutdown():
			
			self.subra=rospy.Subscriber("confirmation_rh", String, self.Rcontrol_arm_callback)
			self.subla=rospy.Subscriber("confirmation_lh", String, self.Lcontrol_arm_callback)
			self.subgr=rospy.Subscriber("confirmation_gr",String, self.Rgrip_callback)
			self.subgl=rospy.Subscriber("confirmation_gl",String, self.Lgrip_callback)
			self.subB=rospy.Subscriber("base_state",String, self.base_callback)
			self.sub3=rospy.Subscriber("/cansPos",Path, self.can_detection_callback)
			self.subh= rospy.Subscriber("/ak_head",Float32, self.head_callback)
			self.subsh= rospy.Subscriber("targetPos",Path,self.shelf_detection_callback)

			self.pubr = rospy.Publisher('rarm',Float32MultiArray, queue_size=10)
			self.publ = rospy.Publisher('larm',Float32MultiArray, queue_size=10)
			self.pub2= rospy.Publisher('gripper', Float32, queue_size=10)
			self.pub4= rospy.Publisher('chatter_1',Float32, queue_size=10)

			if self.state==1:
				print(self.state , self.BarrivalState ,self.cans_detected)
				rospy.sleep(15)
				self.pub2.publish(22)
				rospy.sleep(7)
				while self.BarrivalState== False:
					self.pub4.publish(5.0)

				self.state =2
				
			
			if self.state==2 and self.BarrivalState==True:
			#if self.state==2:
				print(self.state , self.BarrivalState ,self.cans_detected)
				self.pub4.publish(6.0)
				self.state=3

			
			if self.state==3 and self.cans_detected == True and self.head_state==2:
				print(self.state , self.BarrivalState ,self.cans_detected)
				self.state = 4
				self.execute_state=1
				self.cycle= self.cycle+1
				self.cans_detected = False
				# if execute_state==1:
				while (self.done is not True):

							
					if self.execute_state ==1 :
						

						#arm 1
						apose_goal1 = np.ones(7)
						apose_goal1[0]=self.finalPoints.poses[0].position.x-0.3
						apose_goal1[1]=self.finalPoints.poses[0].position.y
						apose_goal1[2]=self.finalPoints.poses[0].position.z
						
						apose_goal1[3]=self.finalPoints.poses[0].orientation.x
						apose_goal1[4]=self.finalPoints.poses[0].orientation.y
						apose_goal1[5]=self.finalPoints.poses[0].orientation.z
						apose_goal1[6]=self.finalPoints.poses[0].orientation.w
						pose_goal1= Float32MultiArray(data =apose_goal1 )
						rospy.sleep(1)
						

						#arm 2
						apose_goal2 = np.ones(7)
						apose_goal2[0]=self.finalPoints.poses[1].position.x-0.3
						apose_goal2[1]=self.finalPoints.poses[1].position.y
						apose_goal2[2]=self.finalPoints.poses[1].position.z
						
						apose_goal2[3]=self.finalPoints.poses[1].orientation.x
						apose_goal2[4]=self.finalPoints.poses[1].orientation.y
						apose_goal2[5]=self.finalPoints.poses[1].orientation.z
						apose_goal2[6]=self.finalPoints.poses[1].orientation.w
						pose_goal2= Float32MultiArray(data =apose_goal2 )
						rospy.sleep(1)	
					
						
						self.publ.publish(pose_goal2)
						print("done publishing goal 2")
						print(pose_goal2)
						self.execute_state = 2

						self.pubr.publish(pose_goal1)
						print("done publishing goal 1")
						print(pose_goal1)


					if self.execute_state == 2 and self.LarmReach==True or self.RarmReach==True:
						self.execute_state = 3
						self.LarmReach=False
						self.LarmReach=False
						rospy.sleep(1)
						self.pub2.publish(11)


					if self.execute_state == 2 and  self.LgripState== True or self.RgripState== True:
							

						#arm 1

						apose_goal1 = np.ones(7)
						pose_goal1[0]=self.finalPoints.poses[0].position.x-0.2
						pose_goal1[1]=self.finalPoints.poses[0].position.y
						pose_goal1[2]=self.finalPoints.poses[0].position.z
						pose_goal1[3]=self.finalPoints.poses[0].orientation.x
						pose_goal1[4]=self.finalPoints.poses[0].orientation.y
						pose_goal1[5]=self.finalPoints.poses[0].orientation.z
						pose_goal1[6]=self.finalPoints.poses[0].orientation.w

						rospy.sleep(1)
						self.pubr.publish(pose_goal1)
						

						#arm 2
						
						apose_goal2 = np.ones(7)
						pose_goal2[0]=self.finalPoints.poses[1].position.x-0.2
						pose_goal2[1]=self.finalPoints.poses[1].position.y
						pose_goal2[2]=self.finalPoints.poses[1].position.z
						pose_goal2[3]=self.finalPoints.poses[1].orientation.x
						pose_goal2[4]=self.finalPoints.poses[1].orientation.y
						pose_goal2[5]=self.finalPoints.poses[1].orientation.z
						pose_goal2[6]=self.finalPoints.poses[1].orientation.w

						rospy.sleep(1)
						self.publ.publish(pose_goal2)
						rospy.sleep(1)
						self.publ.publish(pose_goal2)
						self.done= True
						self.execute_state = 3

					if self.execute_state==3 and self.RarmReach==True and self.LarmReach==True:
						rospy.sleep(1)
						self.pub4.publish(4.0)
						self.RarmReach= False
						self.LarmReach= False
						self.execute_state=4

			
					if self.execute_state==4 and self.BrotateState== True:
						self.BrotateState=False
						self.execute_state=5

					
			
	def head_callback(self, data):
		if data.data==1:
			self.head_state=2

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

		if data.data =="released":
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
		if self.state==3 and self.cans_detected ==False and self.head_state==2:			
			#print(self.cans_detected)
			self.msgcamera_id= data.header.frame_id
			self.msgcamera_poses =data.poses
			print("data",data)
			self.num_cans= len(self.msgcamera_poses)
			campos= np.ones((self.num_cans,3))
			for i in range(self.num_cans):
				tempar= np.ones(3)
				tempar[0]= self.msgcamera_poses[i].pose.position.x
				tempar[1]= self.msgcamera_poses[i].pose.position.y
				tempar[2]= self.msgcamera_poses[i].pose.position.z
				campos[i]=tempar
			
			col_y=campos[np.argsort(campos[:,1])]
			print("col_y",col_y)
			selectedCans =np.ones((2,3))

			if self.num_cans%4== 0:
				first_row= col_y[-4:]
				print("1st row", first_row)
				col_x=first_row[np.argsort(first_row[:,0])]
				print("colx",col_x)
				
				#selectedCans[0,:]= col_x[0]
				#selectedCans[1,:]= col_x[-1]
				selectedCans[0,:]=col_x[1]
				selectedCans[1,:]=col_x[2]
				print("selected cans",selectedCans)

			else:
				first_row= col_y[:2]
				selectedCans= first_row
				# print("selectedCans",selectedCans)

			tfs= PoseArray()
			tfs.header.frame_id= self.msgcamera_id

			tfsp1 = Pose()
			tfsp1.position.x= selectedCans[0][0]
			tfsp1.position.y= selectedCans[0][1]
			tfsp1.position.z= selectedCans[0][2]
			tfsp1.orientation.w = 1
			# print("tfsp1",tfsp1)
			tfs.poses.append(tfsp1)

			tfsp2 = Pose()
			tfsp2.position.x= selectedCans[1][0]
			tfsp2.position.y= selectedCans[1][1]
			tfsp2.position.z= selectedCans[1][2]
			tfsp2.orientation.w=1
			# print("tfsp2",tfsp2)
			tfs.poses.append(tfsp2)
			
			print("before tfs",type(tfs),tfs)

			Trans=TransformServices()
			self.finalPoints = Trans.transform_poses(self.msgcamera_id,'/base_link',tfs)
			
			print("after tfs",type(self.finalPoints),self.finalPoints)

			# print(selectedCans)
			
			# print(data.poses)
			self.cans_detected= True
		
		
	def shelf_detection_callback(self,data):
		if self.execute_state==4 and self.BrotateState== True and self.shelf_detected==False:		
			#print(self.cans_detected)
			self.msgcamera_id= data.header.frame_id
			self.msgcamera_poses =data.poses


			campos= np.ones((self.num_cans,3))
			for i in range(self.num_cans):
				tempar= np.ones(3)
				tempar[0]= self.msgcamera_poses[i].pose.position.x
				
				tempar[1]= self.msgcamera_poses[i].pose.position.ytempar[2]= self.msgcamera_poses[i].pose.position.z
				campos[i]=tempar
			
			selectedCans= np.ones((2,3))
			if self.num_cans==12 or self.num_cans==6:
				col_y=campos[np.argsort(campos[:,1])]
				print("col_y",col_y)
				col_x=col_y[np.argsort(col_y[:3,0])]
				print("col_x",col_x)
				selectedCans = col_x[:2,:]

			elif self.num_cans ==10 or self.num_cans==4:
				col_z=campos[np.argsort(campos[:,2])]
				col_z1 = col_z[-4:,:]
				col_y1=col_z1[np.argsort(col_z1[:,1])]
				selectedCans[0,:] = col_y1[0,:]
				col_y2 = col_y1[1:,:]
				col_x=col_y2[np.argsort(col_y2[:,0])]
				selectedCans[1,:] = col_x[1,:]

			else:
				col_z=campos[np.argsort(campos[:,2])]
				selectedCans = col_z[:-2,:]




			col_y=campos[np.argsort(campos[:,1])]
			print("col_y",col_y)
			selectedCans =np.ones((2,3))

		

			print("selectedCans",selectedCans)
			tfs= PoseArray()
			tfs.header.frame_id= self.msgcamera_id

			tfsp1 = Pose()
			tfsp1.position.x= selectedCans[0][0]
			tfsp1.position.y= selectedCans[0][1]
			tfsp1.position.z= selectedCans[0][2]
			# print("tfsp1",tfsp1)
			tfs.poses.append(tfsp1)

			tfsp2 = Pose()
			tfsp2.position.x= selectedCans[1][0]
			tfsp2.position.y= selectedCans[1][1]
			tfsp2.position.z= selectedCans[1][2]
			# print("tfsp2",tfsp2)
			tfs.poses.append(tfsp2)
			
			print("tfs",type(tfs),tfs)

			Trans=TransformServices()
			self.finalPoints = Trans.transform_poses(self.msgcamera_id,'/base_link',tfs)

			# print(selectedCans)
			
			# print(data.poses)
			self.shelf_detected= True

if __name__=='__main__':

	rospy.init_node("mission_planner")

	
	try:
     		mission_planner()
     	except rospy.ROSInterruptException:
     		pass


	






	


