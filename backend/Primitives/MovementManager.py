import csv
import time


def split_list(robot, primitive_list, pose_time, pose_delay):
    if "," in primitive_list:
        arr = primitive_list.split(", ")
        for prim in arr:
            print(prim)
            if prim == 'null':
                continue
            play_motion(robot, prim, pose_time, pose_delay)
    else:
        play_motion(robot, primitive_list, pose_time, pose_delay)


recorded_poses = []


def play_motion(robot, file_name, pose_time, pose_delay):
    pose_time_millis = int((pose_time - 0.005) * 1000)
    long_file_name = "/Users/caseysnow/Desktop/MQP/flask-project/backend/Primitives/poses/" + file_name
    # long_file_name = "./flaskProject/backend/Primitives/poses/" + file_name
    with open(long_file_name) as f:
        csv_recorded_poses = [{k: int(v) for k, v in row.items()}
                              for row in
                              csv.DictReader(f, skipinitialspace=True)]
    for poseMotorPositionsDict in csv_recorded_poses:
        motor_positions_dict = poseMotorPositionsDict
        robot.update_motors(pose_time_millis, motor_positions_dict)
        time.sleep(pose_time + pose_delay)


def record_motion(robot, pose_num):
    recorded_poses = []
    """
            Records a series of manually positioned robot poses with a desired number of poses and saves them to a csv file
            """
    for m in robot.motors:
        m.compliant_toggle(1)  # sets all motors in the robot to be compliant for moving to poses
        time.sleep(0.05)  # need delay for comm time
    for poseIndex in range(pose_num):  # for each pose from 0 to desired number of poses
        pose_motor_positions_dict = {}
        continue_select = int(input("Type 2 to record to next pose:"))  # wait for user to input "1" in console
        if continue_select != 0:
            time.sleep(0.1)  # delay to allow consistent reading of first motor in first pose
            for m in robot.motors:  # for each motor in Motors list
                pose_motor_positions_dict[
                    m.motor_id] = m.get_position("")  # add the motor ID as key and motor position as value
            recorded_poses.append(pose_motor_positions_dict)  # add dictionary of current robot pose to list of recorded poses
        continue_select = 0
        time.sleep(0.01)  # comms buffer delay
    # write dictionary of recorded poses to csv file
    motorIDHeaders = recorded_poses[0].keys()
    motionFile = open(str(input("Input saved file name:")), "w")  # request a filename
    dict_writer = csv.DictWriter(motionFile, motorIDHeaders)
    dict_writer.writeheader()
    dict_writer.writerows(recorded_poses)
    motionFile.close()
    for m in robot.motors:
        m.compliant_toggle(0)  # set motors back to non-compliant for use elsewhere
        time.sleep(0.05)  # need delay for comm time


def record_motion_ui(robot, file_name, first_time):
    global recorded_poses
    for m in robot.motors:
        m.compliantOnOff(1)  # sets all motors in the robot to be compliant for moving to poses
        time.sleep(0.05)  # need delay for comm time

    pose_motor_positions_dict = {}
    time.sleep(0.1)  # delay to allow consistent reading of first motor in first pose
    for m in robot.motors:  # for each motor in Motors list
        pose_motor_positions_dict[m.motorID] = m.getPosition()  # add the motor ID as key and motor position as
        # value
        recorded_poses.append(pose_motor_positions_dict)  # add dictionary of current robot pose to list of
        # recorded poses

    time.sleep(0.01)  # comms buffer delay
    # write dictionary of recorded poses to csv file
    motor_id_headers = recorded_poses[0].keys()
    with open(file_name, 'a') as file1:
        if first_time:
            file1.writelines(motor_id_headers)
        file1.writelines(recorded_poses)
    file1.close()
    for m in robot.motors:
        m.compliantOnOff(0)  # set motors back to non-compliant for use elsewhere
        time.sleep(0.05)  # need delay for comm time


def play_motion_kinematics(robot, dictList):
    # TODO: integrate this into play_motion?
    """
        iterates through list of recorded poses of entire robot,
        holding each pose for defined pose time.
        """

    # print(csvRecordedPoses)
    for pose_motor_positions_dict in dictList:  # for each pose in the list of recorded poses
        modify_this_position_dict = pose_motor_positions_dict

        print(modify_this_position_dict)
        robot.update_motors(5000, modify_this_position_dict)

        # motorPositionsDict = modify_this_position_dict
        # need to edit the ids of legs but keep everything else to be fed
        # time.sleep(self.poseTime + self.poseDelay)
        time.sleep(.1)
