DEADZONE = 0.2

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

##########################
###  ROBOT COMPONENTS  ###
##########################
controllerConfig = {
    'DRIVER': {
        'ID': 0,
        'DEADZONE': DEADZONE,
        'LEFT_TRIGGER_AXIS': 2,
        'RIGHT_TRIGGER_AXIS': 3,
    },
    'OPERATOR': {
        'ID': 1,
        'DEADZONE': DEADZONE,
        'LEFT_TRIGGER_AXIS': 2,
        'RIGHT_TRIGGER_AXIS': 3,
    }
}

drivetrainConfig = {
    'LEFT': {
        'LEFT_LEADER_ID': 13,
        'LEFT_FOLLOWER_ID': 8,
    },
    'RIGHT': {
        'RIGHT_LEADER_ID': 15,
        'RIGHT_FOLLOWER_ID': 7,
    },
    'DRIVETYPE': ARCADE,
    'OPEN_LOOP_RAMP_RATE': 1.0,
    'ROTATION_CORRECTION': 0.0,
    'COUNTS_PER_REVOLUTION': 80,
    'GEAR_RATIO': 10,
    'WHEEL_CIRCUMFERENCE': 18.84, # 6 inches * pi
    'LEFT_ENCODER': 13,
    'RIGHT_ENCODER': 15, #was 7
    'CLUTCH_MULTIPLIER': 0.65 ,
}

shooterConfig = {
    'SHOOTER_ID': 10,
    'SHOOTER_RPM': 4250,
    'SHOOTER_MAX_RPM': 6000,
    'SHOOTER_MIN_RPM': 0,
}

intakeConfig = {
    # Update IDs when known
    'INTAKE_MOTOR_ID': 17,
    'INTAKE_SOLENOID_FORWARD_ID': 6,
    'INTAKE_SOLENOID_REVERSE_ID': 7,
    'INTAKE_MOTOR_SPEED': 0.8,
}

feederConfig = {
    'FEEDER_ID' : 9,
    'FEEDER_SPEED': 0.8,
}

tiltShooterConfig = {
    'TILTSHOOTER_ID': 1,
    'ROTATIONS_PER_360': 75,
    'MIN_DEGREES': 0,
    'MAX_DEGREES': 35,
    'BUFFER_DEGREES': 2,
    'SPEED': 0.1,
}

aimerConfig = {
    'AIMING_ROTATION_SPEED': 0.6,
    'AIMING_ACCURACY_DEGREES': 1.5,
}

visionConfig = {
    'TARGET_HEIGHT': 8.9,
    'TARGET_RADIUS': 2,
    'SHOOTER_HEIGHT': 3.5,
    'SHOOTER_OFFSET': 1,
    'CAMERA_HEIGHT': 2.9,
    'CAMERA_PITCH': 27,
}

autonConfig = {
    'POSITION': 2, # 1, 2, or 3
    'TILTING_1_TIME': 2,
    'SPINUP_1_TIME': 3,
    'FIRING_1_TIME': 1,
    'ROTATE_1_TIME': 0, #2
    'DRIVE_1_TIME': 0, #0.75
    'INTAKE_TIME': 1,
    'ROTATE_2_TIME': 0,
    'DRIVE_2_TIME': 0, #2
    'TILTING_2_TIME': 1,
    'SPINUP_2_TIME': 1,
    'FIRING_2_TIME': 0.5,
    'ROTATE_SPEED':0.0, #0.2
    'SHOOT_1_SPEED': 0.6,
    'SHOOT_2_SPEED': 0.6,
    'SHOOT_1_RPM': 4150,
    'SHOOT_2_RPM': 4150,
    'DRIVE_SPEED': 0.8, #0.8
    'TILT_1_TARGET_DEGREES': 19, #23
    'POS_1_TILT_2_TARGET_DEGREES': 20, #23
    'POS_1_ROTATE_1_TARGET_DEGREES': -165.5, #-165.5,
    'POS_1_ROTATE_2_TARGET_DEGREES': 165.5, # 165.5,
    'POS_1_DRIVE_1_DISTANCE': 51.2, # 51.2, #inches from front bumper to middle of ball
    'POS_1_DRIVE_2_DISTANCE': 51.2, # 51.2, #inches from front bumper to middle of ball
    'POS_2_TILT_2_TARGET_DEGREES': 25,
    'POS_2_ROTATE_1_TARGET_DEGREES': 147,
    'POS_2_ROTATE_2_TARGET_DEGREES': -147,
    'POS_2_DRIVE_1_DISTANCE': 56.5, # 56.5, #inches from front bumper to middle of ball
    'POS_2_DRIVE_2_DISTANCE': 56.5, # 56.5, #inches from front bumper to middle of ball
    'POS_3_TILT_2_TARGET_DEGREES': 25,
    'POS_3_ROTATE_1_TARGET_DEGREES': 165,
    'POS_3_ROTATE_2_TARGET_DEGREES': -165,
    'POS_3_DRIVE_1_DISTANCE': 51.9, #51.9, #inches from front bumper to middle of ball
    'POS_3_DRIVE_2_DISTANCE': 51.9, #51.9, #inches from front bumper to middle of ball
}

climberConfig = {
    'WINCH_LEFT_ID': 6,
    'WINCH_RIGHT_ID': 14,
    # Pneumatic board IDs
    'SOLENOID_FORWARD_ID': 4,
    'SOLENOID_REVERSE_ID': 5,
    # DIO pin numbers
    'LEFT_LIMIT_ID': 0,
    'RIGHT_LIMIT_ID': 1,
    'CABLE_WRAPPED': 'UNDER',
    # Both speeds positive.
    # Extend speed must be lower than natural extend rate
    'EXTEND_SPEED': 0.4, # Was 0.2, operator requested 0.4
    'RETRACT_SPEED': 0.5,
}

# keys are distances in ft. Values are (v, angle) in rpm and degrees, respectively
autoAimTable = {
    0: (4000, 20),
    1: (4000, 20),
    2: (4000, 20),
    3: (4000, 20),
    4: (4000, 20),
    5: (4050, 20),  # Not tested this and below
    6: (4250, 20),
    7: (4400, 20),
    8: (4050, 23),
    9: (4250, 23),
    10: (4400, 23),
    11: (4500, 23),
    12: (4000, 25),
    13: (4000, 30),
    14: (4200, 30),
    15: (4400, 30), # Not tested here and above
    16: (4500, 30),
    17: (4500, 30),
    18: (4500, 30),
    19: (4500, 30),
    20: (4500, 30),
    21: (4500, 30),
    22: (4500, 30),
    23: (4500, 30),
    24: (4500, 30),
    25: (4500, 30),
    26: (4500, 30),
    27: (4500, 30),
    28: (4500, 30),
    29: (4500, 30),
    30: (4500, 30),
    }

#######################
###  ROBOT CONFIGS  ###
#######################
testbot = { # Always used for unit tests ($ python robot.py sim)
    'CONTROLLERS': controllerConfig,
    'DRIVETRAIN': drivetrainConfig,
    'SHOOTER': shooterConfig,
    'INTAKE': intakeConfig,
    'FEEDER': feederConfig,
    'TILTSHOOTER': tiltShooterConfig,
    'AIMER': aimerConfig,
    'VISION': visionConfig,
    'AUTON': autonConfig,
    'CLIMBER': climberConfig
}

saline = {
    'CONTROLLERS': controllerConfig,
    'DRIVETRAIN': drivetrainConfig,
    'AIMER': aimerConfig,
    'VISION': visionConfig,
    'TILTSHOOTER': tiltShooterConfig,
    'SHOOTER': shooterConfig,
    'INTAKE': intakeConfig,
    'FEEDER': feederConfig,
    'AUTON': autonConfig,
    'CLIMBER': climberConfig,
}

showbot = {
    'CONTROLLERS': controllerConfig,
    'DRIVETRAIN': drivetrainConfig,
    'AIMER': aimerConfig,
    'SHOOTER': shooterConfig,
}
#showbot['SHOOTER']['SHOOTER_ID'] = 10 # how to override just one thing

##########################
###  CONFIG TO DEPLOY  ###
##########################
robotconfig = saline
