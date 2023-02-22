import time

from backend.KoalbyHumaniod.Sensors.PiratedCode.BoardDisplay_EKF import initializeCube, ProjectionViewer
from backend.KoalbyHumaniod.Robot import RealRobot

robot = RealRobot()
while True:
    data = robot.get_tf_luna_data()
    print(data)
