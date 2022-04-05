import roslib
import rospy
import kortex_driver.msg
import kortex_driver.srv
import math

if __name__=='__main__':
    rospy.init_node('gen3_test')
    rospy.wait_for_service('/gen3a/base/send_joint_speeds_command')
    srv_joint_speeds= rospy.ServiceProxy('/gen3a/base/send_joint_speeds_command', kortex_driver.srv.SendJointSpeedsCommand)

    joint_names= ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']


    joint_speed= kortex_driver.msg.JointSpeed()

    joint_speeds = []
    for joint_idx in range(joint_names):
        joint_speed= kortex_driver.msg.JointSpeed()
        joint_speed.joint_identifier= joint_idx
        joint_speed.value= 0.0
        speed_req.input.joint_speeds.append(joint_speed)


        msg = JointSpeed()
        msg.joint_identifier = joint_idx
        msg.value = vel_cmd[joint_idx]
        msg.duration = 0
        joint_speeds.append(msg)
    base_msg.joint_speeds = joint_speeds
    base_msg.duration = 0
    print(base_msg)

    self.arm_cmd_pub.publish(msg)
    rospy.sleep(self.control_period)