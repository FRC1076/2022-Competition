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
    'ROTATION_CORRECTION': 0.0,
    'COUNTS_PER_REVOLUTION': 80,
    'GEAR_RATIO': 10,
    'WHEEL_CIRCUMFERENCE': 8,
    'LEFT_ENCODER': 13,
    'RIGHT_ENCODER': 7,
}

shooterConfig = {
    'SHOOTER_ID': 10,
    'SHOOTER_RPM': 3500,
}

intakeConfig = {
    # Update IDs when known
    'INTAKE_MOTOR_ID': 17,
    'INTAKE_SOLENOID_FORWARD_ID': 1,
    'INTAKE_SOLENOID_REVERSE_ID': 2,
}

feederConfig = {
    'FEEDER_ID' : 9,
    'FEEDER_SPEED': 0.8,
}

tiltShooterConfig = {
    'TILTSHOOTER_ID': 1,
    'ROTATIONS_PER_360': 75,
    'MIN_DEGREES': 5,
    'MAX_DEGREES': 25,
    'BUFFER_DEGREES': 2,
    'SPEED': 0.1,
}

aimerConfig = {
    'AIMING_ROTATION_SPEED': 0.6,
    'AIMING_ACCURACY_DEGREES': 3,
}

visionConfig = {
    'TARGET_HEIGHT': 8.5,
    'TARGET_RADIUS': 2,
    'SHOOTER_HEIGHT': 3.5,
    'SHOOTER_OFFSET': 1,
    'CAMERA_HEIGHT': 4,
    'CAMERA_PITCH': 0,
}

autonConfig = {
    'TILTING_1_TIME': 1,
    'SPINUP_1_TIME': 2,
    'FIRING_1_TIME': 0.5,
    'ROTATE_1_TIME': 2,
    'DRIVE_1_TIME': 2,
    'INTAKE_TIME': 1,
    'ROTATE_2_TIME': 2,
    'DRIVE_2_TIME': 2,
    'TILTING_2_TIME': 1,
    'SPINUP_2_TIME': 1,
    'FIRING_2_TIME': 0.5,
    'TILT_1_TARGET_DEGREES': 20,
    'TILT_2_TARGET_DEGREES': 10,
    'ROTATE_1_TARGET_DEGREES': 90,
    'ROTATE_2_TARGET_DEGREES': 135,
    'SHOOT_SPEED': 0.5,
    'DRIVE_1_SPEED': 0.8,
    'DRIVE_2_SPEED': 0.2,
    'DRIVE_1_DISTANCE': 60, #inches
    'DRIVE_2_DISTANCE': 60, #inches
}

climberConfig = {
    'WINCH_LEFT_ID': 6,
    'WINCH_RIGHT_ID': 14,
    # Pneumatic board IDs
    'SOLENOID_FORWARD_ID': 6,
    'SOLENOID_REVERSE_ID': 0,
    # DIO pin numbers
    'LEFT_LIMIT_ID': 0,
    'RIGHT_LIMIT_ID': 1,
    'CABLE_WRAPPED': 'UNDER',
    # Both speeds positive.
    # Extend speed must be lower than natural extend rate
    'EXTEND_SPEED': 0.2,
    'RETRACT_SPEED': 0.5,
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

gull_lake = {
    'CONTROLLERS': controllerConfig,
    'DRIVETRAIN': drivetrainConfig,
    'AIMER': aimerConfig,
    'VISION': visionConfig,
    'TILTSHOOTER': tiltShooterConfig,
    'SHOOTER': shooterConfig,
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
robotconfig = showbot
