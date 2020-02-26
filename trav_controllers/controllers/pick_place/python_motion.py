#!/usr/bin/env python
# Travis Deegan SDSU Dynamics and Controls Lab 1/28/20
# email: travis.deegan@sdstate.edu
# This program is an example of how to interface the Franka Emika
# Panda arm with move_group_python_interface.
# This information, along with the information from gripper.py, is used
# to build my pick and place controller.


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

xcube_initial = 0.45
ycube_initial = 0
xcube_final = 0.65
ycube_final = 0

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL
    
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link
    
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):

    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -3*pi/4
    joint_goal[4] = 0
    joint_goal[5] = pi/2
    joint_goal[6] = pi/4

    move_group.go(joint_goal, wait=True)

    move_group.stop()
    print joint_goal

  '''
  def move_gripper(self):

    move_group_gripper = self.move_group_gripper

    joint_goal = move_group_gripper.get_current_joint_values
    joint_goal[0] = 0.03
    joint_goal[1] = 0.03
    move_group_gripper.go(joint_goal, wait=True)

    move_group_gripper.stop()
   '''

  def go_to_joint_state1(self):

    move_group = self.move_group

    joint_goal1 = move_group.get_current_joint_values()
    joint_goal1[0] = 0
    joint_goal1[1] = -pi/4
    joint_goal1[2] = 0
    joint_goal1[3] = -3*pi/4
    joint_goal1[4] = 0
    joint_goal1[5] = pi/2
    joint_goal1[6] = pi/4

    move_group.go(joint_goal1, wait=True)

    move_group.stop()


    ## END_SUB_TUTORIAL

  def go_to_pose_goal(self):

    move_group = self.move_group
    pose_goal = geometry_msgs.msg.Pose()

    pose_goal.orientation.w = 0
    pose_goal.orientation.x = -0.383
    pose_goal.orientation.y = -0.924
    pose_goal.orientation.z = 0

    pose_goal.position.x = xcube_initial
    pose_goal.position.y = ycube_initial
    pose_goal.position.z = 0.25
    move_group.set_pose_target(pose_goal)

    ## 0,1,0,0 and 0,0,1,0 provide the same outcome, orienting the eef in the orientation we'd like. 
    ## However, the 7th joint is at like a 45 deg angle and we need to figure out how to control that.
    ## 0.707,0,0.707,0 w,x,y,z corresponds to y = 90 deg Euler angle.

    plan = move_group.go(wait=True)

    move_group.stop()

    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL


  def go_to_pose_goal1(self):

    move_group = self.move_group

    pose_goal1 = geometry_msgs.msg.Pose()
    pose_goal1.orientation.w = 0
    pose_goal1.orientation.x = -0.383
    pose_goal1.orientation.y = -0.924
    pose_goal1.orientation.z = 0
     
    pose_goal1.position.x = xcube_initial
    pose_goal1.position.y = ycube_initial
    pose_goal1.position.z = 0.125
    move_group.set_pose_target(pose_goal1)

    ## 0,1,0,0 and 0,0,1,0 provide the same outcome, orienting the eef in the orientation we'd like. 
    ## However, the 7th joint is at like a 45 deg angle and we need to figure out how to control that.
    ## 0.707,0,0.707,0 w,x,y,z corresponds to y = 90 deg Euler angle.

    plan = move_group.go(wait=True)

    move_group.stop()

    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

  def go_to_pose_goal2(self):

    move_group = self.move_group

    pose_goal2 = geometry_msgs.msg.Pose()
    pose_goal2.orientation.w = 0
    pose_goal2.orientation.x = -0.383
    pose_goal2.orientation.y = -0.924
    pose_goal2.orientation.z = 0
     
    pose_goal2.position.x = xcube_initial
    pose_goal2.position.y = ycube_initial
    pose_goal2.position.z = 0.2
    move_group.set_pose_target(pose_goal2)

    ## 0,1,0,0 and 0,0,1,0 provide the same outcome, orienting the eef in the orientation we'd like. 
    ## However, the 7th joint is at like a 45 deg angle and we need to figure out how to control that.
    ## 0.707,0,0.707,0 w,x,y,z corresponds to y = 90 deg Euler angle.

    plan = move_group.go(wait=True)

    move_group.stop()

    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL




  def go_to_pose_goal3(self):

    move_group = self.move_group
    pose_goal3 = geometry_msgs.msg.Pose()

    pose_goal3.orientation.w = 0
    pose_goal3.orientation.x = -0.383
    pose_goal3.orientation.y = -0.924
    pose_goal3.orientation.z = 0

    pose_goal3.position.x = xcube_final
    pose_goal3.position.y = ycube_final
    pose_goal3.position.z = 0.25
    move_group.set_pose_target(pose_goal3)

    ## 0,1,0,0 and 0,0,1,0 provide the same outcome, orienting the eef in the orientation we'd like. 
    ## However, the 7th joint is at like a 45 deg angle and we need to figure out how to control that.
    ## 0.707,0,0.707,0 w,x,y,z corresponds to y = 90 deg Euler angle.

    plan = move_group.go(wait=True)

    move_group.stop()

    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL


  def go_to_pose_goal4(self):

    move_group = self.move_group

    pose_goal4 = geometry_msgs.msg.Pose()
    pose_goal4.orientation.w = 0
    pose_goal4.orientation.x = -0.383
    pose_goal4.orientation.y = -0.924
    pose_goal4.orientation.z = 0
     
    pose_goal4.position.x = xcube_final
    pose_goal4.position.y = ycube_final
    pose_goal4.position.z = 0.125
    move_group.set_pose_target(pose_goal4)

    ## 0,1,0,0 and 0,0,1,0 provide the same outcome, orienting the eef in the orientation we'd like. 
    ## However, the 7th joint is at like a 45 deg angle and we need to figure out how to control that.
    ## 0.707,0,0.707,0 w,x,y,z corresponds to y = 90 deg Euler angle.

    plan = move_group.go(wait=True)

    move_group.stop()

    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

  def go_to_pose_goal5(self):

    move_group = self.move_group

    pose_goal5 = geometry_msgs.msg.Pose()
    pose_goal5.orientation.w = 0
    pose_goal5.orientation.x = -0.383
    pose_goal5.orientation.y = -0.924
    pose_goal5.orientation.z = 0
     
    pose_goal5.position.x = xcube_final
    pose_goal5.position.y = ycube_final
    pose_goal5.position.z = 0.2
    move_group.set_pose_target(pose_goal5)

    ## 0,1,0,0 and 0,0,1,0 provide the same outcome, orienting the eef in the orientation we'd like. 
    ## However, the 7th joint is at like a 45 deg angle and we need to figure out how to control that.
    ## 0.707,0,0.707,0 w,x,y,z corresponds to y = 90 deg Euler angle.

    plan = move_group.go(wait=True)

    move_group.stop()

    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)
    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()
      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL
  

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal1()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal2()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state1()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal3()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal4()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_pose_goal5()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state1()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
