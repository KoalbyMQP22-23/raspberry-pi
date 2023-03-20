import time

from backend.KoalbyHumaniod.Robot import RealRobot

robot = RealRobot()
robot.close_hand()
time.sleep(100)
robot.open_hand()
time.sleep(100)
robot.stop_hand()
