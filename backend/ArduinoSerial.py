import time
import serial


class ArduinoSerial(object):

    def __init__(self):
        # to run with DXL shield
        # try:
        # self.ser = serial.Serial('/dev/cu.usbserial-14610', 115200, timeout=1)  # change this to get correct port
        #     self.ser = serial.Serial('/dev/cu.usbserial-14610', 115200, timeout=1)  # change this to get correct port
        # except FileNotFoundError:
        #     print("File not found")

        # to run connected to Arduino
        self.ser = serial.Serial('/dev/cu.usbmodem146101', 115200, timeout=1)

        self.ser.reset_input_buffer()
        time.sleep(3)  # serial buffer needs 3-second delay before reading or writing
        # time.sleep sets serial to 0, DO NOT use 0 as a command on arduino side

    def send_command(self, command):  # sends a command to the arduino from the RasPi
        message = str.encode(command)
        self.ser.write(message)

    def read_command(self):  # reads a command from the arduino
        line = self.ser.readline()
        line = line.decode('utf-8').strip()
        return line
