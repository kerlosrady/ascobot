#!/usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_left', anonymous=True)

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
    group_name_larm="arm_left_torso"
    move_group_larm = moveit_commander.MoveGroupCommander(group_name_larm)
    
    group_name_torso= "torso"
    move_group_torso = moveit_commander.MoveGroupCommander(group_name_torso)
    
    group_name_lgrip = "gripper_left"
    move_group_lgrip = moveit_commander.MoveGroupCommander(group_name_lgrip)
    
    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    self.robot=robot
    self.move_group_larm = move_group_larm
    self.move_group_torso = move_group_torso

    self.move_group_lgrip = move_group_lgrip

  def larm_pose_goal(self,x,y,z):
    move_group_larm = self.move_group_larm
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w =-0.00225
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    pose_goal.orientation.x =0.67815
    pose_goal.orientation.y =0.03085
    pose_goal.orientation.z =0.73427
    move_group_larm.clear_pose_targets()
    move_group_larm.set_num_planning_attempts(3)
    move_group_larm.set_goal_tolerance(0.005)
    move_group_larm.set_pose_target(pose_goal,"arm_left_7_link")
    #print("hahah")
    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group_larm.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group_larm.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group_larm.clear_pose_targets()


  def torso_pose_goal(self):
    move_group_torso = self.move_group_torso
    msg = JointState()
    msg.name = ['torso_lift_joint']
    msg.position = [0.2]
    move_group_torso.set_joint_value_target(msg)
    move_group_torso.go()
    ## END_SUB_TUTORI


  def lgrip_pose_goal(self,x,y):
    move_group = self.move_group_lgrip
    msg = JointState()
    msg.name = ['gripper_left_left_finger_joint', 'gripper_left_right_finger_joint']
    msg.position = [x,y]
    move_group.set_joint_value_target(msg)
    move_group.go()
    ## END_SUB_TUTORIA

  def get_or_tol(self):
    move_group = self.move_group_larm
    move_group.set_goal_tolerance(0.005)
    print(move_group.get_goal_orientation_tolerance())
    print(move_group.get_goal_position_tolerance())
    ## END_SUB_TUTORIAL

def callback1(msg):
  pub1 = rospy.Publisher('confirmation_lh', String, queue_size=10)
  x = float(format(msg.data[0], ".4f"))
  y = float(format(msg.data[1], ".4f"))
  z = float(format(msg.data[2], ".4f"))
  tutorial = MoveGroupPythonInterfaceTutorial()
  # tutorial.larm_pose_goal(n_msg.data[0],n_msg.data[1],n_msg.data[2],n_msg.data[3],n_msg.data[4],n_msg.data[5],n_msg.data[6])
  tutorial.larm_pose_goal(x,y,z)
  tutorial.get_or_tol()
  print("larm is moving!!")
  pub1.publish("larm_done")	

# def callback1(msg):
#   n_msg = Float32MultiArray()
#   x = float(format(msg.data[0], ".4f"))
#   y = float(format(msg.data[1], ".4f"))
#   z = float(format(msg.data[2], ".4f"))
#   # n_msg.data = [x, y, z,xx,yy,zz,w]
# #  n_msg.data = [x, y, z]
#   tutorial = MoveGroupPythonInterfaceTutorial()
#   # tutorial.larm_pose_goal(n_msg.data[0],n_msg.data[1],n_msg.data[2],n_msg.data[3],n_msg.data[4],n_msg.data[5],n_msg.data[6])
#   tutorial.larm_pose_goal(x,y,z,xx,yy,w)
#   pub2.publish("larm_done")

def callback2(data):
  if data.data==11:
    tutorial = MoveGroupPythonInterfaceTutorial()
    tutorial.lgrip_pose_goal(0.035,0.035) #gripped
    pub3.publish("gripped")
  
  if data.data==0:
    tutorial = MoveGroupPythonInterfaceTutorial()
    tutorial.lgrip_pose_goal(0.04,0.04) #ungripped
    pub3.publish("released")
  
  if data.data==22:
    tutorial = MoveGroupPythonInterfaceTutorial()
    tutorial.torso_pose_goal()

pub2 = rospy.Publisher('confirmation_lh', String, queue_size=10)
pub3 = rospy.Publisher('confirmation_gl', String, queue_size=10)
arm= rospy.Subscriber('larm', Float32MultiArray, callback1)
grip=rospy.Subscriber('gripper', Float32, callback2)


def main():
  try:
    tutorial = MoveGroupPythonInterfaceTutorial()
    rospy.spin()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

