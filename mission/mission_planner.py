from mission.msg import target_data
from mission.srv import *
import rospy

state=1


class mission_planning():

    def _init_ (self):

        rospy.init_node('mission_planning')
        #rospy.Subscriber("detection", object_pos, vision_callback)

        #rospy.Subscriber("control_base", base_states, control_base_callback)
        self.pub = rospy.Publisher('arm_actions', arm_actions, queue_size=10)
        self.pub2= rospy.Publisher('gripper', bool, queue_size=10)

        #self.pub3= rospy.Publisher('camera_pos', int, queue_size=10)
	
        rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		
		# if state==0 and table_depth >= threshold:
        #     #publish certain action to get back
        #     state=1
        #     msgb= base_data()
        #     msgb.loc= heading_pos
        #     pub1.publish(msgb)

        #if state==1 and table_depth - threshold >=0.1:
        if state==1:
            msgb = base_data()
            msgb.stop =True
            pub1.publish(msgb)
            state =2
            rospy.wait_for_service('can_detection')
            try:
                can_pos = rospy.ServiceProxy('can_detection', can_detection)
                resp1 = visionService(1)
                x1, y1, z1 = resp1.can_loc1
                x2, y2, z2 = resp1.can_loc2
                cans_detected= True

            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

        if state==2 and cans_detected is True:
            state = 3
            execute_state=1
            cycle= cycle+1

            cans_detected= False

        #if execute_state==1:
            while (done is not True)

                if execute_state == 1:
                    
                    # publish(control, coordinates)
                    msg1 = arm_actions()
                    msg1.loc1 = [x1, y1, z1]
                    msg1.loc2 = [x2, y2, z2]
                    pub.publish(msg1)

                if execute_state == 1 and reach_target == True:
                    execute_state = 2
                    reach_target= False
                if execute_state == 2:
                    # publish(gripper)
                    pub2.publish(True)
                    execute_state = 3
                    reach_target=False

                if execute_state == 3 and grip_target == True:
                    # publish(lift)
                    # what are x, y,z for each
                    z1= z1+0.1
                    z2=z2+0.1

                    msg1 = arm_actions()
                    msg1.loc1 = [x1, y1, z1]
                    msg1.loc2 = [x2, y2, z2]

                    pub.publish(msg1)
                    execute_state = 4
                    grip_target= False
                    done= True


                

    
    def grip_callback(data):
        if data.g_state is True:
            grip_target= True
        if data.release_state is True:
            release_target= True

    

            




	


