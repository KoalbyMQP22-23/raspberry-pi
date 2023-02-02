import time
from simulation import sim as vrep


def read_real_tf_data(arduino_serial):
    arduino_serial.send_command("41")
    dist = arduino_serial.read_command.split(":")[1]
    print(dist)
    if dist > 5:
        return 1


def read_sensors(simulation_on, client_id):
    dist = float(vrep.simxGetFloatSignal(client_id, "proximity", vrep.simx_opmode_streaming)[1])
    print(dist)
    if dist > 5:
        print("stop")
