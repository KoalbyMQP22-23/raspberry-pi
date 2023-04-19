import time

from backend.KoalbyHumaniod.Robot import RealRobot
from backend.Primitives.MovementManager import record_motion, Poses


def record_movement(robot, pose_list):
    # robot = RealRobot()  # inits real-world robot
    pose_num = int(input("Input number of poses desired:"))
    record_motion(robot, pose_num, pose_list)  # records motor values corresponding to a position

pose_list = Poses()
robot = RealRobot()
record_movement(robot, pose_list)
print("Done")
