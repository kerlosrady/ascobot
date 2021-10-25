#!/usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

#rospy.init_node("mv_r_node")


class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

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
    
    group_name_larm="arm_left"
    move_group_larm = moveit_commander.MoveGroupCommander(group_name_larm)
    
    group_name_rgrip = "gripper_right"
    move_group_rgrip = moveit_commander.MoveGroupCommander(group_name_rgrip)
    
    group_name_lgrip = "gripper_left"
    move_group_lgrip = moveit_commander.MoveGroupCommander(group_name_lgrip)
    
    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    self.robot=robot
    self.move_group_rarm = move_group_rarm
    self.move_group_larm = move_group_larm
    self.move_group_rgrip = move_group_rgrip
    self.move_group_lgrip = move_group_lgrip


  def rarm_pose_goal(self,x,y,z):
    move_group_rarm = self.move_group_rarm
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w =0.0457
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    pose_goal.orientation.x =0.78539816339
    pose_goal.orientation.y =0
    pose_goal.orientation.z =0.78539816339

    move_group_rarm.set_pose_target(pose_goal,"arm_right_7_link")

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group_rarm.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group_rarm.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group_rarm.clear_pose_targets()

    ## END_SUB_TUTORIAL
    
  def larm_pose_goal(self):
    move_group = self.move_group_larm
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w =1
    pose_goal.position.x = 0.064765
    pose_goal.position.y = 0.83785
    pose_goal.position.z = 0.7151
    pose_goal.orientation.x =0
    pose_goal.orientation.y =0
    pose_goal.orientation.z =1

    move_group.set_pose_target(pose_goal, "arm_left_7_link")

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIA
  
  def rgrip_pose_goal(self):
    move_group = self.move_group_rgrip
    
    msg = JointState()
    msg.name = ['gripper_right_left_finger_joint', 'gripper_right_right_finger_joint']
    msg.position = [0.01,0.01] 

    move_group.set_joint_value_target(msg)
    move_group.go()
    ## END_SUB_TUTORIAL
    
  def lgrip_pose_goal(self):
    move_group = self.move_group_lgrip
    
    msg = JointState()
    msg.name = ['gripper_left_left_finger_joint', 'gripper_left_right_finger_joint']
    msg.position = [0.04,0.04] 

    move_group.set_joint_value_target(msg)
    move_group.go()
    ## END_SUB_TUTORIA
  
 # def head_pose_goal(self):
  #  move_group = self.move_group_head
   # 
    #msg = JointState()
    #msg.name = ['head_1_joint', 'head_2_joint']
    #msg.position = [0,0] 

    #move_group.set_joint_value_target(msg)
    #move_group.go()
    ## END_SUB_TUTORIA

  def callback1(data):
    x=data[0]
    y=data[1]
    z=data[2]
    rarm_pose_goal(Self,x,y,z)


def main():
  try:
    tutorial = MoveGroupPythonInterfaceTutorial()
    #arm= rospy.Subscriber('chatter_2', Float32MultiArray, tutorial.callback1)
    #grip=rospy.Subscriber('chatter_3', Float32, callback2)
    #tutorial = MoveGroupPythonInterfaceTutorial()
    data=Float32MultiArray.data
    data[0]=0.69
    data[1]=-0.137
    data[2]=0.7151
    tutorial.callback1(data)
    #tutorial.rarm_pose_goal()
    #tutorial.larm_pose_goal()
    #tutorial.rgrip_pose_goal()
    #tutorial.lgrip_pose_goal()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return



if __name__ == '__main__':
  main()
