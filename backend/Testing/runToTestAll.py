from threading import Thread
import sys

from backend.KoalbyHumaniod.Robot import RealRobot
from backend.simulation import sim as vrep
from backend.KoalbyHumaniod.Robot import SimRobot
from backend.Primitives.MovementManager import play_motion
# from backend.testing.runToTestSensors import pv

poseTime = float(input("Enter pose time in seconds: "))  # TODO: fix duplicated code
poseDelay = float(input("Enter delay between pose time in seconds: "))
fileName = str(input("Input saved file name to play back: "))
simulationFlag = int(input("Are you running the simulation?? Please enter 1 for yes and 0 for no: "))
if simulationFlag == 1:  # inits simulated robot
    vrep.simxFinish(-1)  # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    print(client_id)  # if 1, then we are connected.

    if client_id != -1:
         print("Connected to remote API server")
    else:
        sys.exit("Not connected to remote API server")

    robot = SimRobot(client_id)  #inits sim robot
    handle = vrep.simxGetObjectHandle(client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]  # gets ID of sim cart
    vrep.simxSetObjectFloatParameter(client_id, handle, vrep.sim_shapefloatparam_mass, 5, vrep.simx_opmode_blocking)  # gives sim cart a mass of 5
else:  # inits real-world robot
    robot = RealRobot()

# play_motion(robot, fileName, poseTime, poseDelay)

# thready = Thread(target=pv.run, args=(robot, simulationFlag, client_id))
# thready.start() # TODO: need to figure out how to actaully thread this. Doesn't start until visualizer board is stopped
thready1 = Thread(target=robot.get_husky_lens_data)  # create thread for HuskyLens operation
thready1.start()  # run HuskyLens thread
