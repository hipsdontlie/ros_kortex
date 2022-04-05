#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectoryPoint

class TorqueCommander():

    def __init__(self, joint1_pub,joint2_pub,joint3_pub,joint4_pub,joint5_pub,joint6_pub,joint7_pub):
        rospy.loginfo("Started torque commander")
        joint_torques_sub = rospy.Subscriber("mpc_torques", JointTrajectoryPoint, self.efforts_callback)
        self.joint1_pub = joint1_pub
        self.joint2_pub = joint2_pub
        self.joint3_pub = joint3_pub
        self.joint4_pub = joint4_pub
        self.joint5_pub = joint5_pub
        self.joint6_pub = joint6_pub
        self.joint7_pub = joint7_pub

    def efforts_callback(self,data):

        joint_torques_all = data.effort
        print(joint_torques_all)
        self.send_torque(self.joint1_pub,joint_torques_all[0])
        self.send_torque(self.joint2_pub,joint_torques_all[1])
        self.send_torque(self.joint3_pub,joint_torques_all[2])
        self.send_torque(self.joint4_pub,joint_torques_all[3])
        self.send_torque(self.joint5_pub,joint_torques_all[4])
        self.send_torque(self.joint6_pub,joint_torques_all[5])
        self.send_torque(self.joint7_pub,joint_torques_all[6])
        

    def send_torque(self,publisher_name,joint_torque):
        publisher_name.publish(joint_torque)
        

    def listener(self):

        r = rospy.Rate(20)
        r.sleep()
        print("here")
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('mpc_torques_commander', anonymous=True)
    joint1_pub = rospy.Publisher("/my_gen3/joint_1_effort_controller/command", Float64,queue_size=1)
    joint2_pub = rospy.Publisher("/my_gen3/joint_2_effort_controller/command", Float64,queue_size=1)
    joint3_pub = rospy.Publisher("/my_gen3/joint_3_effort_controller/command", Float64,queue_size=1)
    joint4_pub = rospy.Publisher("/my_gen3/joint_4_effort_controller/command", Float64,queue_size=1)
    joint5_pub = rospy.Publisher("/my_gen3/joint_5_effort_controller/command", Float64,queue_size=1)
    joint6_pub = rospy.Publisher("/my_gen3/joint_6_effort_controller/command", Float64,queue_size=1)
    joint7_pub = rospy.Publisher("/my_gen3/joint_7_effort_controller/command", Float64,queue_size=1)
    tc = TorqueCommander(joint1_pub,joint2_pub,joint3_pub,joint4_pub,joint5_pub,joint6_pub,joint7_pub)
    while(not rospy.is_shutdown()):
        tc.listener()