#!/usr/bin/env python
import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


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
    
    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    self.robot=robot
    self.move_group_rarm = move_group_rarm
    self.move_group_larm = move_group_larm
    self.move_group_rgrip = move_group_rgrip



  def rarm_pose_goal(self):
    move_group = self.move_group_rarm
    robot=self.robot
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w =1.75395
    pose_goal.position.x = 0.69375
    pose_goal.position.y = -0.13761
    pose_goal.position.z = 0.7151

    move_group.set_pose_target(pose_goal)
    robot.move_group.pick("standard_can_fit_clone_0")

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL
    
  def larm_pose_goal(self):
    move_group = self.move_group_larm
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w =-0.35684
    pose_goal.position.x = 0.064765
    pose_goal.position.y = 0.83785
    pose_goal.position.z = 1.198

    move_group.set_pose_target(pose_goal)

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
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w =0.0011
    pose_goal.position.x = 0.17755
    pose_goal.position.y = -0.20365
    pose_goal.position.z = 0.70258

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIA




def main():
  try:
    tutorial = MoveGroupPythonInterfaceTutorial()
    tutorial.rarm_pose_goal()
    tutorial = MoveGroupPythonInterfaceTutorial()
    tutorial.larm_pose_goal()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return



if __name__ == '__main__':
  main()
