#!/usr/bin/env python
# Travis Deegan SDSU Dynamics and Controls Lab 1/28/20
# email: travis.deegan@sdstate.edu
# This program is an example of how to interface the Franka Emika
# Panda gripper with move_group_python_interface.
# This information, along with the information from python_motion.py, is used
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

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "hand"
    
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
    self.move_group = move_group

  def go_to_joint_state(self):

    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.04
    joint_goal[1] = 0.04

    move_group.go(joint_goal, wait=True)

    move_group.stop()
    print joint_goal

  def go_to_joint_state1(self):

    move_group = self.move_group

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.0
    joint_goal[1] = 0.0

    move_group.go(joint_goal, wait=True)

    move_group.stop()
    print joint_goal

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

    print "============ Press `Enter` to execute a gripper movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state()

    print "============ Press `Enter` to execute a gripper movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state1()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
