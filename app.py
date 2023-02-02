# import null as null
import sys

from flask import Flask, Response, request
from flask import render_template
import xmltodict

from backend.KoalbyHumaniod.Robot import RealRobot, SimRobot
from backend.Primitives.PrimitivesToExecute import PrimitivesToExecute
from backend.Primitives.frontEndTest import frontEndTest
from backend.simulation import sim as vrep
from backend.KoalbyHumaniod.Sensors.sensorData import sensorData

app = Flask(__name__)

robot = None
pte = None
fet = frontEndTest()
sd = sensorData()

WAVE, CLAP, HANDLOOP, WALK = "Wave", "Clap", "Hand Loop", "Walk"
AVAILABLE_COMMANDS = {
    'Wave': WAVE,
    'Clap': CLAP,
    'Hand Loop': HANDLOOP,
    'Walk': WALK
}
BATTERY_LEVEL, IMU = "Battery Level", "IMU"
AVAILABLE_COMMANDS_SENSORS = {
    'Battery Level': BATTERY_LEVEL,
    'IMU': IMU
}


@app.route("/")
def welcome():
    return render_template("welcome.html")


@app.route("/init")
def init():
    global robot
    global pte
    global sensorData
    robot = RealRobot()
    pte = PrimitivesToExecute(robot)
    sd.init_robot(robot)
    return Response("1", mimetype="text/xml")
    # initializes but doesn't change templates - needs to work same a simulation
    # TODO: should be fine now - test it


@app.route("/shutoff")
def shutdown():
    global robot
    robot.shutdown()  # works
    return Response("1", mimetype="text/xml")


@app.route("/init-simulation")
def init_simulation():
    global robot
    global pte
    global sensorData
    vrep.simxFinish(-1)  # just in case, close all opened connections
    client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    print(client_id)  # if 1, then we are connected.

    if client_id != -1:
        print("Connected to remote API server")
    else:
        # sys.exit("Not connected to remote API server")
        print("not connected")
        return Response("0", mimetype="text/xml")

    robot = SimRobot(client_id)
    pte = PrimitivesToExecute(robot)
    sd.init_robot(robot)
    print("button pressed")
    return Response("1", mimetype="text/xml")


@app.route("/home/")
def home():
    return render_template("home.html")


# @app.route("/home/pre_recorded/queue/<cmd>")
# def queue():
#     return render_template("queue.html", commands=AVAILABLE_COMMANDS_QUEUE)


@app.route("/home/pre-recorded/queue")
def queue():
    fet.print_list()
    queue_list = fet.get_queue_list()
    return render_template("queue.html", queueElements=queue_list)


@app.route("/run/<qList>")
def run(qList=None):
    print(qList)
    # return qList
    return Response(qList, mimetype="text/xml")


@app.route("/home/pre-recorded/")
def pre_recorded():
    return render_template("pre-recorded.html", commands=AVAILABLE_COMMANDS)


@app.route("/sensor-data/")
def sensor_data():
    data = sd.get_data()
    return Response(data, mimetype="text/xml")


@app.route('/home/pre-recorded/<cmd>')
def command(cmd=None):
    global pte
    global fet
    # pte = PrimitivesToExecute(robot)
    # pte.add_to_list(cmd)
    # fet.add_to_list(cmd)
    return cmd, 200, {'Content-Type': 'text/plain'}


@app.route("/record_new/")
def record_new():
    return render_template("record_new.html")
