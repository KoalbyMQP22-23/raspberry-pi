# import null as null
import json
import sys

from flask import Flask, Response, request
from flask import render_template

from backend.KoalbyHumaniod.Robot import RealRobot, SimRobot
from backend.Primitives.PrimitivesToExecute import PrimitivesToExecute
from backend.simulation import sim as vrep
from backend.KoalbyHumaniod.Sensors.sensorData import SensorData
from backend.testing import runToTestKinematics
from backend.Primitives import MovementManager
from backend.testing.runToTestKinematics import Walker

app = Flask(__name__)

robot = None
pte = None
sd = SensorData()
client_id = -1
walker = None


@app.route("/")
def welcome():
    return render_template("welcome.html")


@app.route("/init")
def init():
    global robot
    global pte
    global sensor_data
    try:
        robot = RealRobot()
    except FileNotFoundError:
        return Response("0", mimetype="text/xml")
    pte = PrimitivesToExecute(robot)
    sd.init_robot(robot)
    return Response("1", mimetype="text/xml")
    # TODO: fix this. I'm not sure how to get the try catch to work


@app.route("/shutoff")
def shutdown():
    global robot
    robot.shutdown()  # works
    return Response("1", mimetype="text/xml")


@app.route("/init-simulation")
def init_simulation():
    global robot
    global pte
    global sensor_data
    global client_id
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
    return render_template("queue.html")


@app.route("/run", methods=['POST'])
def run():
    global client_id
    content_type = request.headers.get('Content-Type')
    if content_type == 'application/json':
        qList = request.get_json()
        MovementManager.split_list(robot, qList, 1, 1)
        return Response("finished", mimetype="text/xml")
    # return failure
    return Response("failure", mimetype="text/xml")


@app.route("/home/pre-recorded/")
def pre_recorded():
    return render_template("pre-recorded.html")


@app.route("/sensor-data/")
def sensor_data():
    data = sd.get_data()
    json_object = json.dumps(data)
    return Response(json_object, mimetype="text/xml")


@app.route("/rightWalk-start/")
def walk_start():
    global walker
    walker = Walker(True)
    walker.play("rightWalk", 1, 1, robot)
    return Response("Robot finished walking", mimetype="text/xml")

@app.route("/rightWalk-stop/")
def walk_stop():
    global walker
    walker.isWalking = False
    return Response("Robot stopped walking", mimetype="text/xml")

# @app.route('/home/pre-recorded/<cmd>')
# def command(cmd=None):
#     global pte
#     global fet
#     # pte = PrimitivesToExecute(robot)
#     # pte.add_to_list(cmd)
#     # fet.add_to_list(cmd)
#     return cmd, 200, {'Content-Type': 'text/plain'}


@app.route("/record_new/")
def record_new():
    return render_template("record_new.html")
