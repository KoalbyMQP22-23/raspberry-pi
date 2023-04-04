import csv
import sys

from backend.KoalbyHumaniod.Kinematics.TrajectoryPlanning import TrajPlanner
from backend.KoalbyHumaniod.Robot import SimRobot, RealRobot
from backend.Primitives.MovementManager import play_motion_kinematics
from backend.Simulation import sim as vrep


class Walker:
    def __init__(self):
        self.client_id = None
        self.isWalking = None

    def init_sim(self):

        vrep.simxFinish(-1)  # just in case, close all opened connections
        self.client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        if self.client_id != -1:  # TODO: Fix duplicate code
            print("Connected to remote API server")
        else:
            sys.exit("Not connected to remote API server")

    def toggle(self, filename, leg_choice, tf, robot):
        if self.isWalking:
            self.isWalking = False
        else:
            self.isWalking = True
            self.play(filename, leg_choice, tf, robot)

    def play(self, replay_filename, leg_choice, tf, robot):
        traj_planner = TrajPlanner()
        right_leg_positions = []
        left_leg_positions = []
        # TODO: make this not an absolute filepath
        with open(
                "/Users/caseysnow/Desktop/MQP/flask-project/backend/Primitives/poses/" + replay_filename) as f:
            # with open(poses/replay_filename) as f:
            csv_recorded_poses = [{k: int(v) for k, v in row.items()}
                                for row in
                                csv.DictReader(f,
                                               skipinitialspace=True)]  # parses selected csv file into list of poses
        for poseMotorPositionsDict in csv_recorded_poses:  # for each pose in the list of recorded poses
            intermediate_position_left = []
            intermediate_position_right = []
            dummy_position = poseMotorPositionsDict
            for key in dummy_position.keys():
                if key == '13' or key == '14' or key == '15':
                    intermediate_position_left.append(dummy_position[key])
                if key == '17' or key == '19' or key == '20':
                    intermediate_position_right.append(dummy_position[key])
            left_leg_positions.append(intermediate_position_left)
            right_leg_positions.append(intermediate_position_right)
        print(right_leg_positions)
        print(left_leg_positions)

        if leg_choice == 1:
            position_list = left_leg_positions
            keys = [13, 14, 15]
        else:
            position_list = right_leg_positions
            keys = [18, 19, 20]

        t0 = 0
        v0 = 0
        vf = 0

        # numSteps = 0
        while self.isWalking:

            iterate_through_this = traj_planner.execute_cubic_traj(position_list, keys, t0, tf, v0, vf)

            play_motion_kinematics(robot, iterate_through_this)

            if leg_choice == 1:
                leg_choice = 2
                position_list = right_leg_positions
                keys = [18, 19, 20]

            elif leg_choice == 2:
                leg_choice = 1
                position_list = left_leg_positions
                keys = [13, 14, 15]


if __name__ == "__main__":
    sim_flag = float(input("Are you running the simulation? Type 1 for yes and 0 otherwise"))
    walker = Walker(True)
    if sim_flag == 1:
        walker.init_sim()
        robot = SimRobot(walker.client_id)
        handle = vrep.simxGetObjectHandle(walker.client_id, 'Cuboid0', vrep.simx_opmode_blocking)[1]
        vrep.simxSetObjectFloatParameter(walker.client_id, handle, vrep.sim_shapefloatparam_mass, 5,
                                         vrep.simx_opmode_blocking)
    else:
        robot = RealRobot()

    trajPlanner = TrajPlanner()
    replay_filename = str(input("Input saved file name to play back:"))
    legChoice = float(input("Enter 1 to move left leg or 2 to move right leg:"))
    tf = float(input("Enter trajectory time (seconds):"))
    walker.play(replay_filename, legChoice, tf, robot)
    walker.isWalking = bool(input("False to stop walking"))
