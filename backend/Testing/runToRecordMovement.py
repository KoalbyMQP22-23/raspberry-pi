import time

from backend.KoalbyHumaniod.Robot import RealRobot
from backend.Primitives.MovementManager import record_motion


def record_movement(robot):
    time.sleep(1)
    pose_num = int(input("Input number of poses desired:"))
    record_motion(robot, pose_num)


robot = RealRobot()
record_movement(robot)
print("Done")
