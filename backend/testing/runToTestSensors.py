import time

from backend.KoalbyHumaniod.Sensors.PiratedCode.BoardDisplay_EKF import initializeCube, ProjectionViewer
from backend.KoalbyHumaniod.Robot import RealRobot

#Port Finder
import serial.tools.list_ports as ports

com_ports = list(ports.comports())  # create a list of com ['COM1','COM2']
for i in com_ports:
    print(i.device)  # returns 'COMx'

print("a")
robot = RealRobot()

block = initializeCube()
print("before viewer")
pv = ProjectionViewer(640, 480, block)
print("This will go on forever. Simulation and code needs to be manually stopped")
pv.run(robot, "")
