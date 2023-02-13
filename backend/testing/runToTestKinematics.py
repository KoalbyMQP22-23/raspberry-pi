import csv
import sys

from backend.KoalbyHumaniod.Kinematics.TrajectoryPlanning import TrajPlanner
from backend.KoalbyHumaniod.Robot import RealRobot, SimRobot
from backend.Primitives.MovementManager import play_motion_kinematics
from backend.simulation import sim as vrep
from backend.Primitives import poses

global client_id


def init_sim():
    # global client_id
    vrep.simxFinish(-1)  # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:  # TODO: Fix duplicate code
        print("Connected to remote API server")
    else:
        sys.exit("Not connected to remote API server")

    return client_id


# robot = SimRobot(client_id)
#
# handle = vrep.simxGetObjectHandle(client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]
#
# vrep.simxSetObjectFloatParameter(client_id, handle, vrep.sim_shapefloatparam_mass, 5, vrep.simx_opmode_blocking)
#
# trajPlanner = TrajPlanner()


def play(replay_filename, legChoice, tf, client_id):
    robot = SimRobot(client_id)
    trajPlanner = TrajPlanner()
    rightLegPositions = []
    leftLegPositions = []
    # TODO: make this not an absolute filepath
    with open(
            "C:/Users/josh3/Documents/GitHub/flask-project/backend/Primitives/poses/" + replay_filename) as f:
        # with open(poses/replay_filename) as f:
        csvRecordedPoses = [{k: int(v) for k, v in row.items()}
                            for row in
                            csv.DictReader(f, skipinitialspace=True)]  # parses selected csv file into list of poses
    for poseMotorPositionsDict in csvRecordedPoses:  # for each pose in the list of recorded poses
        intermediatePositionLeft = []
        intermediatePositionRight = []
        dummy_position = poseMotorPositionsDict
        for key in dummy_position.keys():
            if key == '13' or key == '14' or key == '15':
                intermediatePositionLeft.append(dummy_position[key])
            if key == '18' or key == '19' or key == '20':
                intermediatePositionRight.append(dummy_position[key])
        leftLegPositions.append(intermediatePositionLeft)
        rightLegPositions.append(intermediatePositionRight)
    print(rightLegPositions)
    print(leftLegPositions)

    if legChoice == 1:
        positionList = leftLegPositions
        keys = [13, 14, 15]
    else:
        positionList = rightLegPositions
        keys = [18, 19, 20]

    t0 = 0
    v0 = 0
    vf = 0

    # walkOn = true
    # while walkOn == true:
    numSteps = 0
    while numSteps <= 6:

        # if UI button pressed:
            # walkOn = false;

        iterateThroughThis = trajPlanner.execute_cubic_traj(positionList, keys, t0, tf, v0, vf)

        play_motion_kinematics(robot, iterateThroughThis)

        if legChoice == 1:
            legChoice = 2
            positionList = rightLegPositions
            keys = [18, 19, 20]

        elif legChoice == 2:
            legChoice = 1
            positionList = leftLegPositions
            keys = [13, 14, 15]

        numSteps = numSteps + 1


if __name__ == "__main__":
    client_id = init_sim()
    # global client_id
    robot = SimRobot(client_id)

    handle0 = vrep.simxGetObjectHandle(client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]
    # handle1 = vrep.simxGetObjectHandle(client_id, 'Pelvis', vrep.simx_opmode_blocking)[1]
    # handle2 = vrep.simxGetObjectHandle(client_id, 'Right_Hip', vrep.simx_opmode_blocking)[1]
    # handle3 = vrep.simxGetObjectHandle(client_id, 'Block0', vrep.simx_opmode_blocking)[1]
    # handle4 = vrep.simxGetObjectHandle(client_id, 'Right_Thigh', vrep.simx_opmode_blocking)[1]
    # handle5 = vrep.simxGetObjectHandle(client_id, 'Right_Shin', vrep.simx_opmode_blocking)[1]
    # handle6 = vrep.simxGetObjectHandle(client_id, 'Right_Foot', vrep.simx_opmode_blocking)[1]
    # handle7 = vrep.simxGetObjectHandle(client_id, 'Left_Hip', vrep.simx_opmode_blocking)[1]
    # handle8 = vrep.simxGetObjectHandle(client_id, 'Block1', vrep.simx_opmode_blocking)[1]
    # handle9 = vrep.simxGetObjectHandle(client_id, 'Left_Thigh', vrep.simx_opmode_blocking)[1]
    # handle10 = vrep.simxGetObjectHandle(client_id, 'Left_Shin', vrep.simx_opmode_blocking)[1]
    # handle11 = vrep.simxGetObjectHandle(client_id, 'Left_Foot', vrep.simx_opmode_blocking)[1]
    # handle12 = vrep.simxGetObjectHandle(client_id, 'Upper_Torso0', vrep.simx_opmode_blocking)[1]
    # handle13 = vrep.simxGetObjectHandle(client_id, 'Lower_Torso', vrep.simx_opmode_blocking)[1]
    # handle14 = vrep.simxGetObjectHandle(client_id, 'Upper_Torso', vrep.simx_opmode_blocking)[1]
    # handle15 = vrep.simxGetObjectHandle(client_id, 'Chest', vrep.simx_opmode_blocking)[1]
    # handle16 = vrep.simxGetObjectHandle(client_id, 'Neck', vrep.simx_opmode_blocking)[1]
    # handle17 = vrep.simxGetObjectHandle(client_id, 'Head', vrep.simx_opmode_blocking)[1]
    # handle18 = vrep.simxGetObjectHandle(client_id, 'Left_Shoulder', vrep.simx_opmode_blocking)[1]
    # handle19 = vrep.simxGetObjectHandle(client_id, 'Cuboid', vrep.simx_opmode_blocking)[1]
    # handle20 = vrep.simxGetObjectHandle(client_id, 'Left_Upper_Arm', vrep.simx_opmode_blocking)[1]
    # handle21 = vrep.simxGetObjectHandle(client_id, 'Left_Forearm', vrep.simx_opmode_blocking)[1]
    # handle22 = vrep.simxGetObjectHandle(client_id, 'Right_Shoulder', vrep.simx_opmode_blocking)[1]
    # handle23 = vrep.simxGetObjectHandle(client_id, 'Block', vrep.simx_opmode_blocking)[1]
    # handle24 = vrep.simxGetObjectHandle(client_id, 'Right_Upper_Arm', vrep.simx_opmode_blocking)[1]
    # handle25 = vrep.simxGetObjectHandle(client_id, 'Right_Forearm', vrep.simx_opmode_blocking)[1]

    vrep.simxSetObjectFloatParameter(client_id, handle0, vrep.sim_shapefloatparam_mass, 5, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle1, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle2, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle3, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle4, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle5, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle6, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle7, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle8, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle9, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle10, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle11, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle12, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle13, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle14, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle15, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle16, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle17, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle18, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle19, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle20, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle21, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle22, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle23, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle24, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectFloatParameter(client_id, handle25, vrep.sim_shapefloatparam_mass, 1, vrep.simx_opmode_blocking)

    trajPlanner = TrajPlanner()
    replay_filename = str(input("Input saved file name to play back:"))
    legChoice = float(input("Enter 1 to move left leg or 2 to move right leg:"))
    tf = float(input("Enter trajectory time (seconds):"))
    play(replay_filename, legChoice, tf, client_id)
