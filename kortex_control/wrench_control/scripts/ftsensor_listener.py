import time
from telnetlib import Telnet

# Initializing Variables
F = [0,0,0]
T = [0,0,0]

# Starting Telnet session
with Telnet('192.168.1.1',23) as tn:
    # Logging into Telnet Spaghetti Code
    read = tn.read_until(b'Login:')
    tn.write(b'ati\n')
    time.sleep(0.2)
    tn.read_until(b'Password: ')
    tn.write(b'ati7720115\n')
    time.sleep(0.2)

    # Turning on the force torque bias
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

        # print(F)
        # print(T)
        print("Fx: " + str(F[0]) + " Fy: " + str(F[1]) +  " Fz: " + str(F[2]) + " Tx: " + str(T[0]) +  " Ty: " + str(T[1]) + " Tz: " + str(T[2]) + " Time: " + str(current_time-previous_time))
        previous_time = current_time