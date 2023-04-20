#!/usr/bin/env python3
"""
Motor 1 - Herkulex, Right Forearm
Motor 2 - Herkulex, Right Upper Arm
Motor 3 - Herkulex, Right Arm Connector
Motor F - Herkulex, Right Shoulder

Motor B - Herkulex, Left Forearm
Motor A - Herkulex, Left Upper Arm
Motor 6 - Herkulex, Left Arm Connector
Motor 7 - Herkulex, Left Shoulder

Motor 11 - Herkulex, Torso Double Rotation Backside
Motor 12 - Herkulex, Torso Double Rotation Frontside
Motor 13 - Herkluex, Abdomen
"""

'''Array of all motors for Koalby.'''
motors = [
# Right Arm
    [1, [0, 0], 'Herk', 'Right_Shoulder_Rotator_Joint'],
    [2, [0, 0], 'Herk', 'Right_Shoulder_Abductor_Joint'],
    [3, [0, 0], 'Herk', 'Right_Upper_Arm_Rotator_Joint'],
    [15, [0, 0], 'Herk', 'Right_Elbow_Joint'],

    # Left Arm
    [11, [0, 0], 'Herk', 'Left_Shoulder_Rotator_Joint'],
    [10, [0, 0], 'Herk', 'Left_Shoulder_Abductor_Joint'],
    [6, [0, 0], 'Herk', 'Left_Upper_Arm_Rotator_Joint'],
    [7, [0, 0], 'Herk', 'Left_Elbow_Joint'],

    # Torso
    [17, [0, 0], 'Herk', 'Lower_Torso_Front2Back_Joint'],
    [18, [0, 0], 'Herk', 'Chest_Side2Side_Joint'],
    [19, [0, 0], 'Herk', 'Upper_Torso_RotatorJoint'],
    [21, [0, 0], 'Herk', 'Lower_Torso_Side2Side_Joint'],
    [23, [0, 0], 'Herk', 'Upper_Torso_Rotator_Joint'],

    # Right Leg
    [9, [0, 0], 'Herk', 'Right_Thigh_Abductor_Joint'],
    [14, [0, 0], 'Herk', 'Right_Thigh_Rotator_Joint'],
    [31, [0, 0], 'Herk', 'Right_Thigh_Kick_Joint'],
    [12, [0, 0], 'Herk', 'Right_Knee_Joint'],
    [5, [0, 0], 'Herk', 'Right_Ankle_Joint'],

    # Left Leg
    [8, [0, 0], 'Herk', 'Left_Thigh_Abductor_Joint'],
    [4, [0, 0], 'Herk', 'Left_Thigh_Rotator_Joint'],
    [30, [0, 0], 'Herk', 'Left_Thigh_Kick_Joint'],
    [20, [0, 0], 'Herk', 'Left_Knee_Joint'],
    [13, [0, 0], 'Herk', 'Left_Ankle_Joint'],

    # Head
    [23, [0, 0], 'Dyn', 'Neck_Forward2Back_Joint'],
    [24, [0, 0], 'Dyn', 'Neck_Rotator_Joint']]
