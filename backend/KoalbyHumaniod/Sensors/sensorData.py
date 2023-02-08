class SensorData(object):
    def __init__(self):
        self.robot = None

    def init_robot(self, robot):
        self.robot = robot

    def get_data(self):
        ret_dict = {"battery_level": (self.robot.read_battery_level()), "imu_data": (self.robot.get_imu_data()),
                    "tf_luna_data": (self.robot.get_tf_luna_data()),
                    "husky_lens_data": (self.robot.get_husky_lens_data())}
        print(ret_dict)
        return ret_dict
