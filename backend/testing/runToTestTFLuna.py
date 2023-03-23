import time

from backend.KoalbyHumaniod.Sensors.PiratedCode.BoardDisplay_EKF import initializeCube, ProjectionViewer
from backend.KoalbyHumaniod.Robot import RealRobot

robot = RealRobot()  # inits real-world robot
while True:
    data = robot.get_tf_luna_data()  # gets TF Luna data
    print(data)
