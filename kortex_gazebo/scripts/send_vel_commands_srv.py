#!/usr/bin/env python
# license removed for brevity

import rospy
from kortex_driver.msg import JointSpeed
from kortex_driver.srv import SendJointSpeedsCommandRequest, SendJointSpeedsCommand
import math

if __name__=='__main__':
  rospy.init_node('gen3_test')
  rospy.wait_for_service('/my_gen3/base/send_joint_speeds_command')
  srv_joint_speeds= rospy.ServiceProxy('/my_gen3/base/send_joint_speeds_command', SendJointSpeedsCommand)
  joint_names= ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

  speed_req = SendJointSpeedsCommandRequest()
  #NOTE: JointSpeed/value is in DEGREES per second.
  #cf. https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/JointSpeed.md
  rad2deg= lambda q:q/math.pi*180.0
  for jidx,jname in enumerate(joint_names):
    joint_speed= JointSpeed()
    joint_speed.joint_identifier= jidx
    joint_speed.value= 0.0
    speed_req.input.joint_speeds.append(joint_speed)

  
  t0= rospy.Time.now()
  rate= rospy.Rate(200)

  try:
    while not rospy.is_shutdown():
      
      t= (rospy.Time.now()-t0).to_sec()
      for joint_speed in speed_req.input.joint_speeds:
        #NOTE: JointSpeed/value is in DEGREES per second.
        if joint_speed.joint_identifier == 3:
          joint_speed.value= -1.0
        else:
          # joint_speed.value= rad2deg(0.5*math.sin(math.pi*t))
          joint_speed.value= 0
      srv_joint_speeds.call(speed_req)
      rate.sleep()

  except KeyboardInterrupt:
      print('Interrupted')

  finally:
    #To make sure the robot stops:
    speed_req= SendJointSpeedsCommandRequest()
    for jidx,jname in enumerate(joint_names):
      joint_speed= JointSpeed()
      joint_speed.joint_identifier= jidx
      joint_speed.value= 0.0
      speed_req.input.joint_speeds.append(joint_speed)
    srv_joint_speeds.call(speed_req)