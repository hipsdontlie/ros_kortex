#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from kortex_driver.msg import Base_JointSpeeds, JointSpeed
from copy import copy
def talker():
    trajectory_pub = rospy.Publisher('my_gen3/in/joint_velocity', Base_JointSpeeds, queue_size = 10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    jtp_speeds = Base_JointSpeeds()
    while not rospy.is_shutdown():
        jtp_speed = JointSpeed()
        jtp_speed.joint_identifier = 0
        jtp_speed.value = 0.01
        jtp_speed.duration = 1
        jtp_speeds.joint_speeds.append(copy(jtp_speed))
        trajectory_pub.publish(jtp_speeds)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass