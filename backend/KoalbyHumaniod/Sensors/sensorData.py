class sensorData(object):
    def __init__(self):
        self.primitives_to_execute = []
        self.robot = None

    def init_robot(self, robot):
        self.robot = robot

    def get_data(self):
        return "data here"
