#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

end_height = 0.1
link3 = 0.128
tojoint3 = 0.13
tojoint2 = 0.043139

rad = math.asin((end_height - tojoint2) / tojoint3)
input = rad + 0.338252

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.move_group.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 1)

rospy.sleep(3)

group_variable_values = group.get_current_joint_values()

group_variable_values[0] = 0
group_variable_values[1] = input
group_variable_values[2] = -input
group.set_joint_value_target(group_variable_values)

plan2 = group.plan()
group.go(wait=True)

rospy.sleep(3)

moveit_commander.roscpp_shutdown()