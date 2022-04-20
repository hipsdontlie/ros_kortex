#!/usr/bin/env python2

import os
import argparse

from torch import cartesian_prod
from arthur_planning.msg import arthur_traj
import rosbag
import rospy

def extract(bagfile, pose_topic, msg_type, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# timestamp tx ty tz qx qy qz qw\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            
                
            cartesian_states = msg.cartesian_states.poses

            for i in range(len(cartesian_states)):
                   
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                        (i+1,
                         cartesian_states[i].position.x, cartesian_states[i].position.y,
                         cartesian_states[i].position.z,
                         cartesian_states[i].orientation.x,
                         cartesian_states[i].orientation.y,
                         cartesian_states[i].orientation.z,
                         cartesian_states[i].orientation.w))
  
            break
    print('wrote ' + str(n) + ' imu messages to the file: ' + out_filename)


if __name__ == '__main__':
    
    rospy.init_node('save_ground_truth')
    parser = argparse.ArgumentParser(description='''
    Extracts IMU messages from bagfile.
    ''')

    bag = 'bags/trajectory_gt.bag'
    msg_type = 'arthur_traj'
    topic = '/my_gen3/arthur_traj'
    output= 'stamped_groundtruth.txt'

    out_dir = os.path.dirname(os.path.abspath(bag))
    out_fn = os.path.join(out_dir, output)

    print('Extract pose from bag '+ bag+' in topic ' +  topic)
    print('Saving to file '+out_fn)
    extract(bag, topic, msg_type, out_fn)
