#!/usr/bin/env python2

import os
import argparse

from torch import cartesian_prod
from geometry_msgs.msg import Pose
import rosbag
import rospy


def extract(bagfile, pose_topic, msg_type, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# timestamp tx ty tz qx qy qz qw\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
                
            print(msg)
 
            f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                    (n+1,
                        msg.position.x, msg.position.y,
                        msg.position.z,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w))
  
            n+=1
    print('wrote ' + str(n) + ' imu messages to the file: ' + out_filename)


if __name__ == '__main__':
    rospy.init_node('save_actual')
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from bagfile.
    ''')

    bag = 'bags/trajectory_actual.bag'
    msg_type = 'Pose'
    topic = '/actual_traj'
    output= 'actual_traj.txt'

    out_dir = os.path.dirname(os.path.abspath(bag))
    out_fn = os.path.join(out_dir, output)

    print('Extract pose from bag '+ bag+' in topic ' +  topic)
    print('Saving to file '+out_fn)
    extract(bag, topic, msg_type, out_fn)
