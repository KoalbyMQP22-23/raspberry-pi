import csv
import time


def play_motion(robot, file_name, pose_time, pose_delay):
    pose_time_millis = int((pose_time - 0.005) * 1000)
    long_file_name = "../Primitives/poses/" + file_name
    with open(long_file_name) as f:
        csv_recorded_poses = [{k: int(v) for k, v in row.items()}
                              for row in
                              csv.DictReader(f, skipinitialspace=True)]
    for poseMotorPositionsDict in csv_recorded_poses:
        motor_positions_dict = poseMotorPositionsDict
        # TODO: kinematics
        robot.update_motors(pose_time_millis, motor_positions_dict)
        time.sleep(pose_time + pose_delay)


def play_motion_kinematics(robot, dictList):
    """
        iterates through list of recorded poses of entire robot,
        holding each pose for defined pose time.
        """

    # print(csvRecordedPoses)
    for poseMotorPositionsDict in dictList:  # for each pose in the list of recorded poses
        modifyThisPositionDict = poseMotorPositionsDict

        print(modifyThisPositionDict)
        robot.update_motors(5000, modifyThisPositionDict)

        # motorPositionsDict = modifyThisPositionDict # need to edit the ids of legs but keep everything else to be fed
        # time.sleep(self.poseTime + self.poseDelay)
        time.sleep(.1)


def recordMotionKinematics(
        self):  # gets positions of leg motors and writes it to CSV file, should be 13, 14, 15, then 18, 19, 20
    """
               Records a series of manually positioned robot poses with a desired number of poses and saves them to a csv file
               """
    self.poseNum = int(input("Input number of poses desired:"))
    for m in self.Motors:
        m.compliantOnOff(1)  # sets all motors in the robot to be compliant for moving to poses
        time.sleep(0.05)  # need delay for comm time
    for poseIndex in range(self.poseNum):  # for each pose from 0 to desired number of poses
        poseMotorPositionsDict = {}
        self.continueSelect = int(
            input("Type 2 to record to next pose:"))  # wait for user to input "1" in console
        if self.continueSelect != 0:
            time.sleep(0.1)  # delay to allow consistent reading of first motor in first pose
            for m in self.Motors:  # for each motor in Motors list
                poseMotorPositionsDict[
                    m.motorID] = m.getPosition()  # add the motor ID as key and motor position as value
            self.recordedPoses.append(
                poseMotorPositionsDict)  # add dictionary of current robot pose to list of recorded poses
        self.continueSelect = 0
        time.sleep(0.01)  # comms buffer delay
    # write dictionary of recorded poses to csv file
    motorIDHeaders = self.recordedPoses[0].keys()
    motionFile = open(str(input("Input saved file name:")), "w")  # request a filename
    dict_writer = csv.DictWriter(motionFile, motorIDHeaders)
    dict_writer.writeheader()
    dict_writer.writerows(self.recordedPoses)
    motionFile.close()
    for m in self.Motors:
        m.compliantOnOff(0)  # set motors back to non-compliant for use elsewhere
        time.sleep(0.05)  # need delay for comm time
