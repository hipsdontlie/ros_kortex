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
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryGoal
from arthur_planning.msg import arthur_traj
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from geometry_msgs.msg import Point, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kortex_driver.msg import Base_JointSpeeds, JointSpeed
import copy
# from arthur_planning.msg import 

class arthur_trajectory(object):

    def __init__(self):
        # Initialize the node and moveit commander
        super(arthur_trajectory, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('pilz_test')
        

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)
        
            self.numTraj = 0
            self.planned = False
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
            # self.execute_plan = actionlib.SimpleActionClient(rospy.get_namespace() + "gen3_joint_trajectory_controller/follow_joint_trajectory/command", FollowJointTrajectoryAction)
            
            print("execute plan server: ", rospy.get_namespace() + "base/play_cartesian_trajectory")

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print (e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def reach_named_position(self, target):
        arm_group = self.arm_group
        
        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        # print(pose)
        rospy.loginfo("Actual cartesian pose is : ")
        # rospy.loginfo(pose.pose)

        return pose.pose

    def reach_cartesian_pose(self, pose, tolerance, constraints):
        arm_group = self.arm_group
        
        # Set the tolerance
        arm_group.set_goal_position_tolerance(tolerance)

        # Set the trajectory constraint if one is specified
        if constraints is not None:
            arm_group.set_path_constraints(constraints)

        # Get the current Cartesian Position
        arm_group.set_pose_target(pose)

        # Plan and execute
        rospy.loginfo("Planning and going to the Cartesian Pose")
        return arm_group.go(wait=True)

    def reach_cartesian_pose_pilz(self, pose, pos_tolerance, orientation_tolerance, constraints):
        cartesian_path = True
        velocity_scaling = 0.1
        acc_scaling = 0.4
        arm_group = self.arm_group
        plan_req = moveit_msgs.msg.MotionPlanRequest()
        plan_req.pipeline_id = "pilz_industrial_motion_planner"
        # plan_req.planner_id = "LIN" if cartesian_path else "PTP"
        plan_req.planner_id = "LIN"
        plan_req.group_name = "arm"
        plan_req.num_planning_attempts = 1
        arthur = arthur_traj()


        mp_req_pose_goal = geometry_msgs.msg.PoseStamped(
            header=std_msgs.msg.Header(frame_id=self.planning_frame), pose=pose)
        # print(mp_req_pose_goal)

        constraints = constructGoalConstraints(
            self.eef_frame, mp_req_pose_goal, pos_tolerance, orientation_tolerance)
        
        print("End effector frame: ", self.eef_frame)

        plan_req.goal_constraints.append(constraints)
        plan_req.max_velocity_scaling_factor = velocity_scaling
        plan_req.max_acceleration_scaling_factor = acc_scaling

        start = time.time()
        mp_res = self.get_plan(plan_req).motion_plan_response
        end = time.time()
        total_time = (end-start)
        print("Time for execution: ", total_time)

        
        if self.planned is True:
            self.numTraj += 1
            arthur.trajNum = self.numTraj
        
        #copying joint states to arthur.traj
        arthur.traj = mp_res.trajectory.joint_trajectory
        
        #populating arthur msg with cartesian states
        for i in range(len(arthur.traj.points)):
            joint_angles = arthur.traj.points[i].positions[:]
            output = self.endEffector_pose(joint_angles)
            if i==0:
                print("End effector pose: ", output)
            velocity = Point()
            # arthur = arthur_traj.cartesian_vel
            if len(arthur.cartesian_states.poses)==0:
                # arthur.cartesian_vel.append([0,0,0])

                velocity.x = 0
                velocity.y = 0
                velocity.z = 0

                arthur.cartesian_vel.append(velocity)

                arthur.cartesian_states.poses.append(output.pose_stamped[0].pose)

            else:
                arthur.cartesian_states.poses.append(output.pose_stamped[0].pose)
                # print(arthur.cartesian_states.poses[i].position.x)
                
                velocity.x = ((arthur.cartesian_states.poses[i].position.x - arthur.cartesian_states.poses[i-1].position.x)/0.1)
                velocity.y = ((arthur.cartesian_states.poses[i].position.y - arthur.cartesian_states.poses[i-1].position.y)/0.1)
                velocity.z = ((arthur.cartesian_states.poses[i].position.z - arthur.cartesian_states.poses[i-1].position.z)/0.1)

                arthur.cartesian_vel.append(velocity)

        print("Num Waypoints ", len(arthur.cartesian_states.poses))

        self.arthur_traj_pub(arthur)      

        return True

#arthur msg publisher
    def arthur_traj_pub(self, arthur):
        pub = rospy.Publisher('arthur_traj', arthur_traj, queue_size=10)
        rate = rospy.Rate(10) # 10hz

        # while not rospy.is_shutdown():
        while self.planned is True:
            self.planned = False
            # rospy.loginfo("Publishing sensor msg")
            pub.publish(arthur)
            rate.sleep()


#calculating end-effector pose using FK
    def endEffector_pose(self, joint_angles):

        compute_fk_srv = rospy.ServiceProxy("/my_gen3/compute_fk", GetPositionFK)
        # print("waiting for service")
        rospy.wait_for_service("/my_gen3/compute_fk")
        # print("found service")
        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = "base_link"
        fk_request.fk_link_names = ['tool_frame']
        fk_request.robot_state.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        fk_request.robot_state.joint_state.position = joint_angles
        
        output = compute_fk_srv(fk_request)

        return output

#trajectory following using joint velocities (not used)
    def execute_trajectory(self, arthur):
        # controller_name = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory/command/goal'
        controller_name = '/my_gen3/in/joint_velocity'

        # trajectory_pub = rospy.Publisher(controller_name, FollowJointTrajectoryActionGoal, queue_size = 10)
        trajectory_pub = rospy.Publisher(controller_name, Base_JointSpeeds, queue_size = 10)

        jtp_speeds = Base_JointSpeeds()
        jtp_list = []
        print("Publishing trajectory execution")


        for i in range(len(arthur.traj.points)):
            # jtp_speeds = Base_JointSpeeds()
            for j in range(7):
                jtp_speed = JointSpeed()
                jtp_speed.joint_identifier = j
                jtp_speed.value = arthur.traj.points[i].velocities[j]
                jtp_speed.duration = 1
                jtp_speeds.joint_speeds.append(copy.copy(jtp_speed))

            print(jtp_speeds)
        print("Executed trajectory")

    def set_pose_callback(self, end_point, args):
        # rospy.loginfo("Entered set pose")
        example = args[0]
        success = args[1]
        actual_pose = example.get_cartesian_pose()
        self.reaming_end_point = end_point
        print("Self.planned: ", self.planned)
        if self.planned is True:
            rospy.loginfo("In set pose callback")
            actual_pose.position.x = self.reaming_end_point.pose.position.x
            actual_pose.position.y = self.reaming_end_point.pose.position.y
            actual_pose.position.z = self.reaming_end_point.pose.position.z
            rospy.loginfo("Going to plan trajectory")
            success &= example.reach_cartesian_pose_pilz(pose=actual_pose, pos_tolerance=0.01, orientation_tolerance=0.005, constraints=None)
            self.planned = False

    # def error_callback(self, error, args):
    #     self.error_bool = error
    #     # rospy.loginfo("Error present: %s", self.error_bool)
    #     # if self.error_bool.data is True:
    #     #     self.planned = False
    #     #     rospy.loginfo("Planned changed to false in DC")

    #     if self.error_bool.data is True:  #and self.planned is False:
    #         rospy.loginfo("In dynamic compensation")
    #         self.new_end_point = self.reaming_end_point
    #         example = args[0]
    #         success = args[1]
    #         actual_pose = example.get_cartesian_pose()
    #         actual_pose.position.x = self.reaming_end_point.pose.position.x
    #         actual_pose.position.y = self.reaming_end_point.pose.position.y
    #         actual_pose.position.z = self.reaming_end_point.pose.position.z
    #         success &= example.reach_cartesian_pose_pilz(pose=actual_pose, pos_tolerance=0.01, orientation_tolerance=0.005, constraints=None)
    #         # self.planned = True

    def start_planning_callback(self, data):
        self.planned = data.data


def main():
    example = arthur_trajectory()

  # For testing purposes
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

    if success:
        rospy.loginfo("Welcome to main :)")
        # rospy.Subscriber("/pelvis_error", std_msgs.msg.Bool, example.error_callback, (example, success))
        rospy.Subscriber("/start_planning", std_msgs.msg.Bool, example.start_planning_callback, queue_size=10)
        rospy.Subscriber("/reaming_end_point", PoseStamped, example.set_pose_callback, (example, success))

        rospy.loginfo("Reaching Cartesian Pose...")
        print (success)

    rospy.spin()
    

  # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
    rospy.init_node('pilz_test')
    main()