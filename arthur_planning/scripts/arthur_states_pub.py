import ros
import tf
import sys
import time
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg 
import std_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from moveit.core.kinematic_constraints import constructGoalConstraints
from moveit_msgs.srv import GetMotionPlan
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from arthur_planning.msg import arthur_traj
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from geometry_msgs.msg import Point