from backend.KoalbyHumaniod.Robot import RealRobot
import serial.tools.list_ports as ports

# com_ports = list(ports.comports())  # create a list of com ['COM1','COM2']
# for i in com_ports:
#     print(i.device)  # returns 'COMx' / current COM port being used

robot = RealRobot()  # inits real-world robot
# shut down robot
