#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from kortex_driver.msg import Base_JointSpeeds, JointSpeed
import copy

#trajectory following using joint velocities (not used)
def send_vel_commands():
    rospy.init_node('velocity_commander', anonymous=True)
    controller_name = '/my_gen3/in/joint_velocity'

    # trajectory_pub = rospy.Publisher(controller_name, FollowJointTrajectoryActionGoal, queue_size = 10)
    trajectory_pub = rospy.Publisher(controller_name, Base_JointSpeeds, queue_size = 10)

    jtp_speeds = Base_JointSpeeds()
    jtp_list = []
    print("Publishing trajectory execution")

    # jtp_speeds = Base_JointSpeeds()
    while not rospy.is_shutdown():
        for j in range(7):
            jtp_speed = JointSpeed()
            jtp_speed.joint_identifier = j
            jtp_speed.value = 0.1
            jtp_speed.duration = 1
            jtp_speeds.joint_speeds.append(copy.copy(jtp_speed))

        print(jtp_speeds)
        trajectory_pub.publish(jtp_speeds)
        print("Executed trajectory")

if __name__ == '__main__':
    try:
        send_vel_commands()
    except rospy.ROSInterruptException:
        pass