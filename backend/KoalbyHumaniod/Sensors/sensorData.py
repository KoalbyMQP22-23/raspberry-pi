class SensorData(object):
    def __init__(self):
        self.robot = None

    def init_robot(self, robot):
        self.robot = robot

    def get_data(self):
        self.robot.get_imu_data()
        self.robot.read_battery_level()
        self.robot.get_tf_luna_data()
        self.robot.get_husky_lens_data()
        # TODO: make returns into JSON or something useful
        return "data here"

