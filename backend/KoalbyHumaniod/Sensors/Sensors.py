import time

from simple_pid import PID

from backend.simulation import sim as vrep


def get_sim_imu_data(client_id):
    ret_data = []
    ret_data.append(vrep.simxGetFloatSignal(client_id, "gyroX", vrep.simx_opmode_streaming)[1])
    ret_data.append(vrep.simxGetFloatSignal(client_id, "gyroY", vrep.simx_opmode_streaming)[1])
    ret_data.append(vrep.simxGetFloatSignal(client_id, "gyroZ", vrep.simx_opmode_streaming)[1])
    ret_data.append(vrep.simxGetFloatSignal(client_id, "accelerometerX", vrep.simx_opmode_streaming)[1])
    ret_data.append(vrep.simxGetFloatSignal(client_id, "accelerometerY", vrep.simx_opmode_streaming)[1])
    ret_data.append(vrep.simxGetFloatSignal(client_id, "accelerometerZ", vrep.simx_opmode_streaming)[1])
    ret_data.append(1)
    ret_data.append(1)
    ret_data.append(1)

    return ret_data


def do_work(yaw, pitch, roll, client_id):
    # PID control
    pass
    # pitch_pid = PID(.5, 0, 0, setpoint=4.4)
    # # pitch_pid.output_limits = (-.25, .75)  # Output value will be between 0 and 10
    # # # while True:
    # # # Compute new output from the PID according to the systems current value
    # print("pitch angle is {}".format(pitch))
    # # print(pitch)
    #
    # handle = vrep.simxGetObjectHandle(client_id, 'Lower_Torso_Front2Back_Joint', vrep.simx_opmode_blocking)[1]
    #
    # motor_pos = vrep.simxGetJointPosition(client_id, handle, vrep.simx_opmode_streaming)[1]
    # print("Motor position is {}".format(motor_pos))
    #
    # control = pitch_pid(pitch)
    # print("control is {}".format(control))
    #
    # vrep.simxSetJointTargetPosition(client_id, handle, control, vrep.simx_opmode_streaming)

