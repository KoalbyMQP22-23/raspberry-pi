from threading import Thread

from backend.Testing.runToTestMovement import setup
from backend.Primitives.MovementManager import play_motion
from backend.Testing.runToTestSensors import pv

robot, file_name, pose_time, pose_delay, client_id, simulation_flag = setup()
movement_thread = Thread(target=play_motion, args=(robot, file_name, pose_time, pose_delay))
thready = Thread(target=pv.run, args=(robot, simulation_flag, client_id))
