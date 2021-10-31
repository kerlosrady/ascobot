#!/usr/bin/env python

import rospy
import sys
import copy
from std_msgs.msg import String
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import JointState

#rospy.init_node("mv_rr_node")


class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('move_arm_right', anonymous=True)
    #rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name_rarm = "arm_right_torso"
    move_group_rarm = moveit_commander.MoveGroupCommander(group_name_rarm)
    
    group_name_rgrip = "gripper_right"
    move_group_rgrip = moveit_commander.MoveGroupCommander(group_name_rgrip) 

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    self.robot=robot
    self.move_group_rarm = move_group_rarm
    self.move_group_rgrip = move_group_rgrip


  def rarm_pose_goal(self,x,y,z,dd):
    move_group_rarm = self.move_group_rarm
    if dd==0:
      print("up")
      pose_goalup = geometry_msgs.msg.Pose() 
      pose_goalup.orientation.w =0.0563
      pose_goalup.position.x = x
      pose_goalup.position.y = y
      pose_goalup.position.z = 0.9
      pose_goalup.orientation.x =0.66329
      pose_goalup.orientation.y =-0.017027
      pose_goalup.orientation.z =0.74605
      move_group_rarm.set_pose_target(pose_goalup,"arm_right_7_link")

    ## Now, we call the planner to compute the plan and execute it.
      plan = move_group_rarm.go(wait=True)

      print("close_arm")
      pose_goalcl = geometry_msgs.msg.Pose() 
      pose_goalcl.orientation.w =0.0563
      pose_goalcl.position.x = 0.10494
      pose_goalcl.position.y = -0.59317
      pose_goalcl.position.z = 1.0042
      pose_goalcl.orientation.x =0.66329
      pose_goalcl.orientation.y =-0.017027
      pose_goalcl.orientation.z =0.74605
      move_group_rarm.set_pose_target(pose_goalcl,"arm_right_7_link")

    ## Now, we call the planner to compute the plan and execute it.
      plan = move_group_rarm.go(wait=True)

      pub5 = rospy.Publisher('chatter_1', Float32, queue_size=10)
      pub5.publish(4.0)

      
    if dd==1:
      pose_goal1 = geometry_msgs.msg.Pose() 
      pose_goal1.orientation.w =0.0563
      pose_goal1.position.x = 0.27
      pose_goal1.position.y = -0.35
      pose_goal1.position.z = 0.9
      pose_goal1.orientation.x =0.66329
      pose_goal1.orientation.y =-0.017027
      pose_goal1.orientation.z =0.74605

      pose_goal2 = geometry_msgs.msg.Pose()
      pose_goal2.orientation.w =0.0563
      pose_goal2.position.x = 0.27
      pose_goal2.position.y = y
      pose_goal2.position.z = 0.9
      pose_goal2.orientation.x =0.66329
      pose_goal2.orientation.y =-0.017027
      pose_goal2.orientation.z =0.74605

      pose_goal3 = geometry_msgs.msg.Pose()
      pose_goal3.orientation.w =0.0563
      pose_goal3.position.x = 0.27
      pose_goal3.position.y = y
      pose_goal3.position.z = z
      pose_goal3.orientation.x =0.66329
      pose_goal3.orientation.y =-0.017027
      pose_goal3.orientation.z =0.74605

      pose_goal4 = geometry_msgs.msg.Pose()
      pose_goal4.orientation.w =0.0563
      pose_goal4.position.x = x
      pose_goal4.position.y = y
      pose_goal4.position.z = z
      pose_goal4.orientation.x =0.66329
      pose_goal4.orientation.y =-0.017027
      pose_goal4.orientation.z =0.74605
      move_group_rarm.set_pose_target(pose_goal1,"arm_right_7_link")

      ## Now, we call the planner to compute the plan and execute it.
      plan = move_group_rarm.go(wait=True)
      print("pose1 done")
      #rospy.sleep(3)

      move_group_rarm.set_pose_target(pose_goal2,"arm_right_7_link")

      ## Now, we call the planner to compute the plan and execute it.
      plan = move_group_rarm.go(wait=True)
      print("pose2 done")
      #rospy.sleep(3)

      move_group_rarm.set_pose_target(pose_goal3,"arm_right_7_link")

      ## Now, we call the planner to compute the plan and execute it.
      plan = move_group_rarm.go(wait=True)
      print("pose3 done")
      #rospy.sleep(3)

      move_group_rarm.set_pose_target(pose_goal4,"arm_right_7_link")

      ## Now, we call the planner to compute the plan and execute it.
      plan = move_group_rarm.go(wait=True)
      print("pose4 done")
      #rospy.sleep(3)
    if dd==2:
        pose_goal5 = geometry_msgs.msg.Pose()
        pose_goal5.orientation.w =0.0563
        pose_goal5.position.x = 0.10494
        pose_goal5.position.y = -0.2
        pose_goal5.position.z = z
        pose_goal5.orientation.x =0.66329
        pose_goal5.orientation.y =-0.017027
        pose_goal5.orientation.z =0.74605

        pose_goal55 = geometry_msgs.msg.Pose()
        pose_goal55.orientation.w =0.0563
        pose_goal55.position.x = 0.10494
        pose_goal55.position.y = y
        pose_goal55.position.z = z
        pose_goal55.orientation.x =0.66329
        pose_goal55.orientation.y =-0.017027
        pose_goal55.orientation.z =0.74605

        pose_goal555 = geometry_msgs.msg.Pose()
        pose_goal555.orientation.w =0.0563
        pose_goal555.position.x = x
        pose_goal555.position.y = y
        pose_goal555.position.z = z
        pose_goal555.orientation.x =0.66329
        pose_goal555.orientation.y =-0.017027
        pose_goal555.orientation.z =0.74605
        move_group_rarm.set_pose_target(pose_goal5,"arm_right_7_link")

      ## Now, we call the planner to compute the plan and execute it.
        plan = move_group_rarm.go(wait=True)
        print("pose1 done")
        #rospy.sleep(3)

        move_group_rarm.set_pose_target(pose_goal55,"arm_right_7_link")

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group_rarm.go(wait=True)
        print("pose2 done")
        #rospy.sleep(3)

        move_group_rarm.set_pose_target(pose_goal555,"arm_right_7_link")

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group_rarm.go(wait=True)
        print("pose3 done")
        #rospy.sleep(3)
        move_group_rarm.set_goal_tolerance(0.01)


        #move_group_rarm.set_pose_target(pose_goal2,"arm_right_7_link")
        
        ## Now, we call the planner to compute the plan and execute it.
        #plan = move_group_rarm.go(wait=True)
        #print("pose2 done")
        #rospy.sleep(3)
        
        # Calling `stop()` ensures that there is no residual movement
        move_group_rarm.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group_rarm.clear_pose_targets()
        ## END_SUB_TUTORIAL

  def rgrip_pose_goal(self,x,y):
    move_group = self.move_group_rgrip
    msg = JointState()
    msg.name = ['gripper_right_left_finger_joint', 'gripper_right_right_finger_joint']
    msg.position = [x,y] 
    move_group.set_joint_value_target(msg)
    move_group.go()
    ## END_SUB_TUTORIAL
    
# def callback1(data):
#   x = format(data.data[0], ".2f")
#   y = format(data.data[1], ".2f")
#   z = format(data.data[2], ".2f")
#   tutorial = MoveGroupPythonInterfaceTutorial()
#   tutorial.rarm_pose_goal(x,y,z)
# x,y,z=0,0,0

def callback1(msg):
  pub1 = rospy.Publisher('confirmation_rh', String, queue_size=10)
  n_msg = Float32MultiArray()
  global x,y,z
  x = float(format(msg.data[0], ".3f"))
  y = float(format(msg.data[1], ".3f"))
  z = float(format(msg.data[2], ".3f"))
  idd=msg.data[3]
  print("x: ",x , "y:  ", y, "z:  ",z)
  n_msg.data = [x, y, z]
  tutorial = MoveGroupPythonInterfaceTutorial()
  tutorial.rarm_pose_goal(n_msg.data[0],n_msg.data[1],n_msg.data[2],idd)
  print("rarm is moving!!")
  pub1.publish("rarm_done")

def callback2(data):
  global x,y,z
  pub3 = rospy.Publisher('confirmation_gr', String, queue_size=10)
  if data.data==11:
    tutorial = MoveGroupPythonInterfaceTutorial()
    tutorial.rgrip_pose_goal(0.03,0.03) #gripped
    pub3.publish("gripped")
  if data.data==0:
    tutorial = MoveGroupPythonInterfaceTutorial()
    tutorial.rgrip_pose_goal(0.04,0.04) #ungripped
    pub3.publish("released")
  if data.data==333:
    tutorial = MoveGroupPythonInterfaceTutorial()
    tutorial.rarm_pose_goal(x,y,z,0) #move arm up

def main():
  try:
    pub1 = rospy.Publisher('confirmation_rh', String, queue_size=10)
    pub3 = rospy.Publisher('confirmation_gr', String, queue_size=10)
    tutorial = MoveGroupPythonInterfaceTutorial()
    arm= rospy.Subscriber('rarm', Float32MultiArray, callback1)
    grip=rospy.Subscriber('gripper', Float32, callback2)
    pub5 = rospy.Publisher('chatter_1', Float32, queue_size=10)
    rospy.spin()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()


