from backend.KoalbyHumaniod.Robot import RealRobot
from backend.Primitives.MovementManager import record_motion


def record_movement():
    robot = RealRobot
    record_motion(robot)


record_movement()
print("Done")
