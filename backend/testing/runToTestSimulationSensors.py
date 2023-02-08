import sys
from time import sleep

from backend.KoalbyHumaniod.Sensors.PiratedCode.BoardDisplay_EKF import initializeCube, ProjectionViewer
from backend.simulation import sim as vrep
from backend.KoalbyHumaniod.Robot import SimRobot


def run_sim_sensors():
    vrep.simxFinish(-1)  # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:  # TODO: Fix duplicate code
        print("Connected to remote API server")
    else:
        sys.exit("Not connected to remote API server")

    robot = SimRobot(client_id)
    handle = vrep.simxGetObjectHandle(client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]
    vrep.simxSetObjectFloatParameter(client_id, handle, vrep.sim_shapefloatparam_mass, 5, vrep.simx_opmode_blocking)
    # vrep.simxSetObjectParent('Right_Forearm', handle, False)
    # vrep.simxSetObjectParent('Left_Forearm', handle, False)
    block = initializeCube()
    pv = ProjectionViewer(640, 480, block)
    print("This will go on forever. Simulation and code needs to be manually stopped")
    pv.run(robot, client_id)
