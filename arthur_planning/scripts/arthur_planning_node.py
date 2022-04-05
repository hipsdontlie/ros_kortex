#!/usr/bin/env python3
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
# from arthur_planning.msg import 

class arthur_trajectory(object):

    def __init__(self):
        # Initialize the node and moveit commander
        super(arthur_trajectory, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pilz_test')
        

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)
        
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            
            # print("move group len")
            # print(len(self.arm_group))
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
            self.planning_frame = self.arm_group.get_planning_frame()
            self.eef_frame = self.arm_group.get_end_effector_link()
            self.get_plan = rospy.ServiceProxy(rospy.get_namespace() + "plan_kinematic_path", GetMotionPlan)
            self.execute_plan = actionlib.SimpleActionClient(rospy.get_namespace() + "gen3_joint_trajectory_controller/follow_joint_trajectory/command", FollowJointTrajectoryAction)
            

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            print("Initializing node in namespace " + rospy.get_namespace())

            print("============ Planning frame: %s" % self.planning_frame)

            # We can also print the name of the end-effector link for this group:
            print("============ End effector link: %s" % self.eef_frame)

            # We can get a list of all the groups in the robot:
            group_names = self.robot.get_group_names()
            print("============ Available Planning Groups:", self.robot.get_group_names())

            # Sometimes for debugging it is useful to print the entire state of the
            # robot:
            print("============ Printing robot state")
            print(self.robot.get_current_state())
            print("")
        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True


    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        print(pose)
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose_pilz(self, pose, pos_tolerance, orientation_tolerance, constraints):
        cartesian_path = True
        velocity_scaling = 0.2
        acc_scaling = 0.1
        arm_group = self.arm_group
        plan_req = moveit_msgs.msg.MotionPlanRequest()
        plan_req.pipeline_id = "pilz_industrial_motion_planner"
        # plan_req.planner_id = "LIN" if cartesian_path else "PTP"
        plan_req.planner_id = "LIN"
        plan_req.group_name = "arm"
        plan_req.num_planning_attempts = 1

        mp_req_pose_goal = geometry_msgs.msg.PoseStamped(
            header=std_msgs.msg.Header(frame_id=self.planning_frame), pose=pose)
        print(mp_req_pose_goal)

        constraints = constructGoalConstraints(
            self.eef_frame, mp_req_pose_goal, pos_tolerance, orientation_tolerance)

        plan_req.goal_constraints.append(constraints)
        plan_req.max_velocity_scaling_factor = velocity_scaling
        plan_req.max_acceleration_scaling_factor = acc_scaling

        mp_res = self.get_plan(plan_req).motion_plan_response

        arthur = arthur_traj()
        
        # arthur.traj.points.resize(2)
        print(mp_res.trajectory.joint_trajectory.points[0].positions[:])
        print(type([mp_res.trajectory.joint_trajectory.points[0].positions[:]]))

        # point = arthur.traj.points
        # point.positions.append(mp_res.trajectory.joint_trajectory.points[0].positions[:])
        # point.append(mp_res.trajectory.joint_trajectory.points[0].velocities[:])
        arthur.traj = mp_res.trajectory.joint_trajectory
        
        self.tf_endEffector(arthur)
        self.arthur_traj_pub(arthur)

        joint_traj = mp_res.trajectory
        print("Type: ",type(mp_res))

        display_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory

        display_trajectory.trajectory_start = mp_res.trajectory_start
        traj = display_trajectory.trajectory

            
        if mp_res.error_code.val != mp_res.error_code.SUCCESS:
            rospy.logerr("Planner failed to generate a valid plan to the goal pose")
            return False
        if (len(mp_res.trajectory.joint_trajectory.points) > 1 and
            mp_res.trajectory.joint_trajectory.points[-1].time_from_start
            == mp_res.trajectory.joint_trajectory.points[-2].time_from_start
        ):
            mp_res.trajectory.joint_trajectory.points.pop(-2)
            rospy.logwarn(
                "Duplicate time stamp in the planned trajectory. Second last way-point was removed."
            )
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal(trajectory=mp_res.trajectory)
        self.execute_plan.send_goal(goal)
        rospy.loginfo("Send goal to the trajectory server successfully!")
        self.execute_plan.wait_for_result()

        return True


    def arthur_traj_pub(self, arthur):
        pub = rospy.Publisher('arthur_traj', arthur_traj, queue_size=10)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            rospy.loginfo("Publishing sensor msg")
            pub.publish(arthur)
            rate.sleep()

    def tf_endEffector(self, arthur):
        tf_listener_ = tf.TransformListener()
        # if tf.frameExists("/base_link") and tf.frameExists("/end_effector_link"):
        # t = tf_listener_.getLatestCommonTime("/base_link", "/end_effector_link")

        tf_listener_.waitForTransform("/base_link", "/end_effector_link", rospy.Time(), rospy.Duration(3.0))
        (trans,rot) = tf_listener_.lookupTransform('base_link', 'end_effector_link', rospy.Time(0))
        print(type(trans))
        print(rot)
        arthur.cartesian_states.position = trans
        print(arthur.cartesian_states)
        arthur.cartesian_states.orientation = rot
        print(arthur.cartesian_states)


def main():
    example = arthur_trajectory()

    # For testing purposes
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass



    if success:
        rospy.loginfo("Reaching Cartesian Pose...")
        
        actual_pose = example.get_cartesian_pose()
        actual_pose.position.z -= 0.2
        actual_pose.position.y -= 0
        actual_pose.position.x -= 0
        success &= example.reach_cartesian_pose_pilz(pose=actual_pose, pos_tolerance=0.01, orientation_tolerance=0.005, constraints=None)
        print (success)
        

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")


if __name__ == '__main__':
    main()