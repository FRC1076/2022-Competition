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
    'FRONTLEFT_DRIVEMOTOR': 1,
    'FRONTRIGHT_DRIVEMOTOR': 2,
    'REARLEFT_DRIVEMOTOR': 3,
    'REARRIGHT_DRIVEMOTOR': 4,
    'FRONTLEFT_ROTATEMOTOR': 5,
    'FRONTRIGHT_ROTATEMOTOR': 6,
    'REARLEFT_ROTATEMOTOR': 7,
    'REARRIGHT_ROTATEMOTOR': 8,
    'DRIVETYPE': SWERVE,
    'ROTATION_CORRECTION': 0.0,
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
    'TILTING_TIME': 2,
    'SPINUP_TIME': 3,
    'FIRING_TIME': 3,
    'BACKUP_TIME': 1.5,
    'TILT_TARGET_DEGREES': 20,
    'SHOOT_SPEED': 0.5,
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
    'VISION': visionConfig,
    'SHOOTER': shooterConfig,
}
showbot['SHOOTER']['SHOOTER_ID'] = 10 # how to override just one thing

##########################
###  CONFIG TO DEPLOY  ###
##########################
robotconfig = gull_lake
