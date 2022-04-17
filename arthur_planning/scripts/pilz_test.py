#!/usr/bin/env python3
from doctest import Example
from multiprocessing.dummy import shutdown
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
        rospy.loginfo(pose.pose)

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

        arthur = arthur_traj()
        if self.error_pose==True:
            arthur.trajNum +=1
        
        # arthur.traj.points.resize(2)
        # print(mp_res.trajectory.joint_trajectory.points[0].positions[:])
        # print(type([mp_res.trajectory.joint_trajectory.points[0].positions[:]]))

        # point = arthur.traj.points
        # point.positions.append(mp_res.trajectory.joint_trajectory.points[0].positions[:])
        # point.append(mp_res.trajectory.joint_trajectory.points[0].velocities[:])
        arthur.traj = mp_res.trajectory.joint_trajectory
        # print("Trajectory result from MI: ", mp_res)
        # print(arthur.traj)
        # print((arthur.traj.points[-1].positions[:]))
        

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

                # arthur.cartesian_vel.poses[0].position.x = 0
                # arthur.cartesian_vel.poses[0].position.y = 0
                # arthur.cartesian_vel.poses[0].position.z = 0

            else:
                arthur.cartesian_states.poses.append(output.pose_stamped[0].pose)
                # print(arthur.cartesian_states.poses[i].position.x)
                
                velocity.x = ((arthur.cartesian_states.poses[i].position.x - arthur.cartesian_states.poses[i-1].position.x)/0.1)
                velocity.y = ((arthur.cartesian_states.poses[i].position.y - arthur.cartesian_states.poses[i-1].position.y)/0.1)
                velocity.z = ((arthur.cartesian_states.poses[i].position.z - arthur.cartesian_states.poses[i-1].position.z)/0.1)

                arthur.cartesian_vel.append(velocity)

                # arthur.cartesian_vel.poses[0].position.x = ((arthur.cartesian_states.poses[i].position.x - arthur.cartesian_states.poses[i-1].position.x)/0.1)
                # arthur.cartesian_vel.poses[0].position.y = ((arthur.cartesian_states.poses[i].position.y - arthur.cartesian_states.poses[i-1].position.y)/0.1)
                # arthur.cartesian_vel.poses[0].position.z = ((arthur.cartesian_states.poses[i].position.z - arthur.cartesian_states.poses[i-1].position.z)/0.1)
                # arthur.cartesian_vel.append([((arthur.cartesian_states.poses[i].position.x - arthur.cartesian_states.poses[i-1].position.x)/0.1),
                #                             ((arthur.cartesian_states.poses[i].position.y - arthur.cartesian_states.poses[i-1].position.y)/0.1),
                #                             ((arthur.cartesian_states.poses[i].position.z - arthur.cartesian_states.poses[i-1].position.z)/0.1)])

            # else:
            #     arthur.cartesian_states.poses[i] = output.pose_stamped[0].pose
            # print(arthur.cartesian_states)
        # self.tf_endEffector(arthur)
        # print(type(arthur.cartesian_vel))
        # print(type(arthur.cartesian_vel[0]))
        print("Num Waypoints ", len(arthur.cartesian_states.poses))
        # print(arthur.traj.points[0])
        # print(arthur.cartesian_states.poses[0])
        # print(arthur.cartesian_vel[0])
        # print(arthur.traj.points[1])
        # print(arthur.cartesian_states.poses[1])
        # print(arthur.cartesian_vel[1])

        # goal = moveit_msgs.msg.ExecuteTrajectoryGoal(trajectory=mp_res.trajectory)
        # self.execute_plan.send_goal(goal)
        # rospy.loginfo("Send goal to the trajectory server successfully!")
        # self.execute_plan.wait_for_result()

        

        # print("Message: ", arthur)
        # self.execute_trajectory(arthur)
        self.arthur_traj_pub(arthur)

        # joint_traj = mp_res.trajectory
        # print("Type: ",type(mp_res))

        # display_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        # display_trajectory = moveit_msgs.msg.DisplayTrajectory

        # display_trajectory.trajectory_start = mp_res.trajectory_start
        # traj = display_trajectory.trajectory

            
        # if mp_res.error_code.val != mp_res.error_code.SUCCESS:
        #     rospy.logerr("Planner failed to generate a valid plan to the goal pose")
        #     return False
        # if (len(mp_res.trajectory.joint_trajectory.points) > 1 and
        #     mp_res.trajectory.joint_trajectory.points[-1].time_from_start
        #     == mp_res.trajectory.joint_trajectory.points[-2].time_from_start
        # ):
        #     mp_res.trajectory.joint_trajectory.points.pop(-2)
        #     rospy.logwarn(
        #         "Duplicate time stamp in the planned trajectory. Second last way-point was removed."
        #     )
        # goal = moveit_msgs.msg.ExecuteTrajectoryGoal(trajectory=mp_res.trajectory)
        # self.execute_plan.send_goal(goal)
        # rospy.loginfo("Send goal to the trajectory server successfully!")
        # self.execute_plan.wait_for_result()
        

        return True

    def arthur_traj_pub(self, arthur):
        pub = rospy.Publisher('arthur_traj', arthur_traj, queue_size=10)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            # rospy.loginfo("Publishing sensor msg")
            pub.publish(arthur)
            rate.sleep()

    def endEffector_pose(self, joint_angles):
        # tf_listener_ = tf.TransformListener()
        # if tf.frameExists("/base_link") and tf.frameExists("/end_effector_link"):
        # t = tf_listener_.getLatestCommonTime("/base_link", "/end_effector_link")


        # tf_listener_.waitForTransform("/base_link", "/end_effector_link", rospy.Time(), rospy.Duration(3.0))
        # (trans,rot) = tf_listener_.lookupTransform('base_link', 'end_effector_link', rospy.Time(0))
        # print(type(trans))
        # print(rot)
        # arthur.cartesian_states.position = trans
        # print(arthur.cartesian_states)
        # arthur.cartesian_states.orientation = rot
        # print(arthur.cartesian_states)
        compute_fk_srv = rospy.ServiceProxy("/my_gen3/compute_fk", GetPositionFK)
        # print("waiting for service")
        rospy.wait_for_service("/my_gen3/compute_fk")
        # print("found service")
        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = "base_link"
        # fk_request.header.stamp = rospy.Time.now()
        fk_request.fk_link_names = ['tool_frame']
        fk_request.robot_state.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        fk_request.robot_state.joint_state.position = joint_angles
        
        output = compute_fk_srv(fk_request)

        return output


    def execute_trajectory(self, arthur):
        # controller_name = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory/command/goal'
        controller_name = '/my_gen3/in/joint_velocity'

        # trajectory_pub = rospy.Publisher(controller_name, FollowJointTrajectoryActionGoal, queue_size = 10)
        trajectory_pub = rospy.Publisher(controller_name, Base_JointSpeeds, queue_size = 10)

        # print(arthur)
        # print("Joint angles: ", arthur.traj.points[0].positions[:])
        # print("Joint angles: ", arthur.traj.points[1].positions[:])
        # print("Joint angles: ", arthur.traj.points[4].positions[:])
        jtp_speeds = Base_JointSpeeds()
        # jtp_speeds = Base_JointSpeeds()
        # jtp.goal.trajectory = arthur.traj
        # jtp.header.stamp = rospy.Time()
        # jtp.header.frame_id = 'base_link'
        # print(arthur.traj.points)
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
        # while not rospy.is_shutdown():
        #     trajectory_pub.publish(jtp_speeds)

            # print("original value")
            # print(jtp_speed)
            # print("After append")
            # jtp_list.append(copy.copy(jtp_speed))
            # print(jtp_list)
        
        # trajectory_pub.publish(jtp_speeds)
            
        
        # print(jtp_speed)
        # print(jtp_list)
        # print(jtp_speeds)


            # jtp.joint_speeds = arthur.traj.points[i].velocities
            # jtp.duration = 1
            # print("Joint step ", i, "=", jtp)
            # trajectory_pub.publish(jtp)
        # jtp.joint_speeds = arthur.traj.points.velocities

        # print(jtp)
        
        # trajectory_pub.publish(jtp)
        print("Executed trajectory")

        # for i in range(len(arthur.traj.points)):
        #     jtp = JointTrajectory()
        #     jtp.points[0].positions = arthur.traj.points[i].positions[:]
        #     jtp.points[0].velocities = arthur.traj.points[i].velocities[:]
        #     trajectory_pub.publish(jtp)

    def set_pose_callback(self, end_point, args):
        rospy.loginfo("Entered set pose")
        example = args[0]
        success = args[1]
        actual_pose = example.get_cartesian_pose()
        self.reaming_end_point = end_point
        # actual_pose.position.z = 0.218
        # actual_pose.position.y = 0.742
        # actual_pose.position.x = 0.036
        if self.error_pose==False:
            actual_pose.position.x = self.reaming_end_point.pose.position.x
            actual_pose.position.y = self.reaming_end_point.pose.position.y
            actual_pose.position.z = self.reaming_end_point.pose.position.z
            rospy.loginfo("Going to plan trajectory")
            success &= example.reach_cartesian_pose_pilz(pose=actual_pose, pos_tolerance=0.01, orientation_tolerance=0.005, constraints=None)

    def error_callback(self, error, args):
        self.error_pose = error

        if self.error_pose==True:
            self.new_end_point = self.reaming_end_point
            example = args[0]
            success = args[1]
            actual_pose = example.get_cartesian_pose()
            actual_pose.position.x = self.reaming_end_point.pose.position.x
            actual_pose.position.y = self.reaming_end_point.pose.position.y
            actual_pose.position.z = self.reaming_end_point.pose.position.z
            success &= example.reach_cartesian_pose_pilz(pose=actual_pose, pos_tolerance=0.01, orientation_tolerance=0.005, constraints=None)
            





def main():
    example = arthur_trajectory()

  # For testing purposes
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass

#   if success:
#     rospy.loginfo("Reaching Named Target Vertical...")
#     success &= example.reach_named_position("vertical")
#     print (success)
  
#   if success:
#     rospy.loginfo("Reaching Joint Angles...")  
#     success &= example.reach_joint_angles(tolerance=0.01) #rad
#     print (success)
  
#   if success:
#     rospy.loginfo("Reaching Named Target Home...")
#     success &= example.reach_named_position("home")
#     print (success)

#   if success:
#     rospy.loginfo("Reaching Named Target Retract...")
#     success &= example.reach_named_position("retract")
#     print (success)



#   if success:
#     rospy.loginfo("Reaching Cartesian Pose...")
    
#     actual_pose = example.get_cartesian_pose()
#     actual_pose.position.z -= 0.2
#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
#     print (success)

    if success:
        rospy.loginfo("Welcome to main :)")
        # rospy.Subscriber("/reaming_end_point", PoseStamped, set_pose_callback, (example, success))
        # rospy.Subscriber("error", Pose, callback)
        actual_pose = example.get_cartesian_pose()  
        actual_pose.position.z += 0.05
        actual_pose.position.y -= 0.1
        actual_pose.position.x += 0.05
        success &= example.reach_cartesian_pose_pilz(pose=actual_pose, pos_tolerance=0.01, orientation_tolerance=0.005, constraints=None)



        rospy.loginfo("Reaching Cartesian Pose...")
        print (success)

    
    # actual_pose = example.get_cartesian_pose()
    # # actual_pose.position.z = 0.218
    # # actual_pose.position.y = 0.742
    # # actual_pose.position.x = 0.036
    # actual_pose.position.z += 0.0
    # actual_pose.position.y -= 0.1
    # actual_pose.position.x += 0.0
    # success &= example.reach_cartesian_pose_pilz(pose=actual_pose, pos_tolerance=0.01, orientation_tolerance=0.005, constraints=None)
    rospy.spin()
    
#   if example.degrees_of_freedom == 7 and success:
#     rospy.loginfo("Reach Cartesian Pose with constraints...")
#     # Get actual pose
#     actual_pose = example.get_cartesian_pose()
#     actual_pose.position.y -= 0.3
    
#     # Orientation constraint (we want the end effector to stay the same orientation)
#     constraints = moveit_msgs.msg.Constraints()
#     orientation_constraint = moveit_msgs.msg.OrientationConstraint()
#     orientation_constraint.orientation = actual_pose.orientation
#     constraints.orientation_constraints.append(orientation_constraint)

#     # Send the goal
#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

#   if example.is_gripper_present and success:
#     rospy.loginfo("Opening the gripper...")
#     success &= example.reach_gripper_position(0)
#     print (success)

#     rospy.loginfo("Closing the gripper 50%...")
#     success &= example.reach_gripper_position(0.5)
#     print (success)

  # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
    rospy.init_node('pilz_test')
    main()