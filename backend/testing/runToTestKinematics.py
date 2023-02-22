import csv
import sys

from backend.KoalbyHumaniod.Kinematics.TrajectoryPlanning import TrajPlanner
from backend.KoalbyHumaniod.Robot import SimRobot
from backend.Primitives.MovementManager import play_motion_kinematics
from backend.simulation import sim as vrep


class Walker:
    def __init__(self, isWalking):
        self.client_id = None
        self.isWalking = isWalking

    def init_sim(self):

        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if self.client_id != -1:  # TODO: Fix duplicate code
            print("Connected to remote API server")
        else:
            sys.exit("Not connected to remote API server")

    # def play(self, replay_filename, legChoice, tf, robot):
    # def play(self, replay_filename, legChoice, tf, robot):
    def play(self, legChoice, tf, robot):
        # robot = SimRobot(self.client_id)
        # trajPlanner = TrajPlanner()
        numTimes = 0
        rightLegPositions = []
        leftLegPositions = []
        rightLegPositions2 = []
        leftLegPositions2 = []
        while(numTimes < 2):
            # TODO: make this not an absolute filepath
            if numTimes == 0:
                with open(
                        "C:/Users/josh3/Documents/GitHub/flask-project/backend/Primitives/poses/leftWalk") as f:
                        # "C:/Users/josh3/Documents/GitHub/flask-project/backend/Primitives/poses/" + replay_filename) as f:
                    # with open(poses/replay_filename) as f:
                    csvRecordedPoses = [{k: int(v) for k, v in row.items()}
                                        for row in
                                        csv.DictReader(f,
                                                       skipinitialspace=True)]  # parses selected csv file into list of poses
            if numTimes == 1:
                with open(
                        "C:/Users/josh3/Documents/GitHub/flask-project/backend/Primitives/poses/rightWalk") as f:
                    # "C:/Users/josh3/Documents/GitHub/flask-project/backend/Primitives/poses/" + replay_filename) as f:
                    # with open(poses/replay_filename) as f:
                    csvRecordedPoses = [{k: int(v) for k, v in row.items()}
                                        for row in
                                        csv.DictReader(f,
                                                       skipinitialspace=True)]  # parses selected csv file into list of poses
            for poseMotorPositionsDict in csvRecordedPoses:  # for each pose in the list of recorded poses
                intermediatePositionLeft = []
                intermediatePositionRight = []
                dummy_position = poseMotorPositionsDict
                for key in dummy_position.keys():
                    if key == '13' or key == '14' or key == '15':
                        intermediatePositionLeft.append(dummy_position[key])
                    if key == '18' or key == '19' or key == '20':
                        intermediatePositionRight.append(dummy_position[key])
                if numTimes == 0:
                    leftLegPositions.append(intermediatePositionLeft)       # step with left leg
                    rightLegPositions.append(intermediatePositionRight)
                if numTimes == 1:
                    leftLegPositions2.append(intermediatePositionLeft)      # step with right leg
                    rightLegPositions2.append(intermediatePositionRight)
            numTimes = numTimes + 1

        print("Right then left 1 then 2")
        print(rightLegPositions)
        print(leftLegPositions)
        print(rightLegPositions2)
        print(leftLegPositions2)


        if legChoice == 1:
            legChoice = 2
            positionList = leftLegPositions
            positionList2 = rightLegPositions
            keys = [13, 14, 15]
            keys2 = [18, 19, 20]
        else:
            legChoice = 1
            positionList = rightLegPositions2
            positionList2 = leftLegPositions2
            keys = [18, 19, 20]
            keys2 = [13, 14, 15]

        t0 = 0
        v0 = 0
        vf = 0

        # numSteps = 0
        while self.isWalking:
            # numSteps <= 6:
            print("New Step")
            iterateThroughThis = trajPlanner.execute_cubic_traj(positionList, keys, t0, tf, v0, vf)
            iterateThroughThis2 = trajPlanner.execute_cubic_traj(positionList2, keys2, t0, tf, v0, vf)
            iterateThroughThisFinal = []

            for i in range(len(iterateThroughThis)-1):
                iterateThroughThisFinal.append(iterateThroughThis[i])
                iterateThroughThisFinal.append(iterateThroughThis2[i])
            # iterateThroughThisFinal = []

            print("iterateThroughThisFinal")
            print(iterateThroughThisFinal)
            play_motion_kinematics(robot, iterateThroughThis)

            # play_motion_kinematics(robot, iterateThroughThisFinal)
            # play_motion_kinematics(robot, iterateThroughThis2)

            # play_motion_kinematics(robot, iterateThroughThisFinal)

            if legChoice == 1:
                legChoice = 2
                positionList = leftLegPositions
                positionList2 = rightLegPositions
                keys = [13, 14, 15]
                keys2 = [18, 19, 20]
            # else:
            #     legChoice = 1
            #     positionList = rightLegPositions2
            #     positionList2 = leftLegPositions2
            #     keys = [18, 19, 20]
            #     keys2 = [13, 14, 15]


if __name__ == "__main__":
    walker = Walker(True)
    walker.init_sim()
    robot = SimRobot(walker.client_id)
    handle = vrep.simxGetObjectHandle(walker.client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]
    vrep.simxSetObjectFloatParameter(walker.client_id, handle, vrep.sim_shapefloatparam_mass, 5,
                                     vrep.simx_opmode_blocking)

    # vrep.setArrayParameter(sim.arrayparam_gravity, {gx, gy, gz})
    vrep.simxSetObjectFloatParameter(walker.client_id, handle, vrep.sim_shapefloatparam_mass, 5,
                                     vrep.simx_opmode_blocking)

    trajPlanner = TrajPlanner()
    # replay_filename = str(input("Input saved file name to play back:"))
    legChoice = float(input("Enter 1 to move left leg or 2 to move right leg:"))
    tf = float(input("Enter trajectory time (seconds):"))
    # walker.play(replay_filename, legChoice, tf, robot)
    walker.play(legChoice, tf, robot)
    walker.isWalking = bool(input("False to stop walking"))
