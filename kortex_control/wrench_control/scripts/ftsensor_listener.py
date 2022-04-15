#!/usr/bin/env python
import time
from telnetlib import Telnet
import rospy
from geometry_msgs.msg import Vector3
import numpy as np

# Initializing Variables
Force = Vector3()
Torque = Vector3()

def getFTinfo():
    force_pub = rospy.Publisher('sensor_force',Vector3,queue_size=10)
    torque_pub = rospy.Publisher('sensor_torque',Vector3,queue_size=10)
    rospy.init_node('ftsensor_listener',anonymous=True)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        # Starting Telnet session
        with Telnet('192.168.1.1',23) as tn:
            # Logging into Telnet Spaghetti Code
            print("Connected to telnet session")
            F = [0,0,0]
            T = [0,0,0]
            read = tn.read_until(b'Login:')
            tn.write(b'ati\n')
            time.sleep(0.2)
            tn.read_until(b'Password: ')
            tn.write(b'ati7720115\n')
            time.sleep(0.2)

            # Turning on the force torque bias
            print("Logged in")
            tn.read_until(b'Type help for commands\r\n>')
            tn.write(b'BIAS ON\r')
            blah = tn.read_until(b'BIAS ON\r\nBIAS ON\r\n',timeout=1)
            current_time = time.time()
            previous_time = time.time()

            # Looping through and getting the force and torque values
            while True:
                current_time = time.time()
                # Get the force xyz and torque xyz
                tn.write(b'S fxyztxyz\r')
                tn.read_until(b'\n')

                # Read in values
                values = str(tn.read_until(b'\n',timeout = 1))
                # print(values)

                # Parse force data
                for i in range(0,3):
                    if i == 0:
                        values = values[2:-1]
                    F[i] = float(values.partition("N")[0].strip())
                    values = values.partition("N")[2].strip()

                # Parse torque data
                for i in range(0,3):
                    T[i] = float(values.partition("Nm")[0].strip())
                    values = values.partition("Nm")[2].strip()

                print("Fx: " + str(F[0]) + " Fy: " + str(F[1]) +  " Fz: " + str(F[2]) + " Tx: " + str(T[0]) +  " Ty: " + str(T[1]) + " Tz: " + str(T[2]) + " Time: " + str(current_time-previous_time))
                Force.x = np.float64(F[0])
                Force.y = np.float64(F[1])
                Force.z = np.float64(F[2])
                Torque.x = np.float64(T[0])
                Torque.y = np.float64(T[1])
                Torque.z = np.float64(T[2])
                force_pub.publish(Force)
                torque_pub.publish(Torque)
                rate.sleep()


if __name__ == '__main__':
    try:
        print("Starting node")
        getFTinfo()
    except rospy.ROSInterruptException:
        pass