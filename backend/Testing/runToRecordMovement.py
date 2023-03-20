from backend.KoalbyHumaniod.Robot import RealRobot
from backend.Primitives.MovementManager import record_motion


def record_movement():
    robot = RealRobot
    pose_num = int(input("Input number of poses desired:"))
    record_motion(robot, pose_num)


record_movement()
print("Done")
