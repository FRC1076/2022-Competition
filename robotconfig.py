DEADZONE = 0.2

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

competition = {
    'CONTROLLERS': {
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
    },
    'DRIVETRAIN': {
        'LEFT': {
            'LEFT_LEADER_ID': 1,
            'LEFT_FOLLOWER_ID': 2,
        },
        'RIGHT': {
            'RIGHT_LEADER_ID': 3,
            'RIGHT_FOLLOWER_ID': 4,
        },
        'DRIVETYPE': ARCADE
    },
    'SHOOTER': {
        'SHOOTER_ID': 5,
        'SHOOTER_RPM': 1000,
    },
    'INTAKE': {
        # Update IDs when known
        'INTAKE_MOTOR_ID': 6,
        'INTAKE_SOLENOID_FORWARD_ID': 1,
        'INTAKE_SOLENOID_REVERSE_ID': 2,
    },
    'FEEDER': {
        'FEEDER_ID' : 9,
        'FEEDER_SPEED': 0.8,
    },
    'TILTSHOOTER': {
        'TILTSHOOTER_ID': 10,
        'ROTATIONS_PER_360': 75,
        'MIN_DEGREES': 0,
        'MAX_DEGREES': 25,
        'BUFFER_DEGREES': 2,
        'SPEED': 0.1,
    },
    'AIMER': {
        'AIMING_ROTATION_SPEED': 0.6,
        'AIMING_ACCURACY_DEGREES': 3,
    },
    'VISION': {
        'TARGET_HEIGHT': 8.5,
        'TARGET_RADIUS': 2,
        'SHOOTER_HEIGHT': 3.5,
        'SHOOTER_OFFSET': 1,
        'CAMERA_HEIGHT': 4,
        'CAMERA_PITCH': 0,
    },
    'CLIMBER': {
        'WINCH_LEFT_ID': 7,
        'WINCH_RIGHT_ID': 8,
        # Pneumatic board IDs
        'SOLENOID_FORWARD_ID': 3,
        'SOLENOID_REVERSE_ID': 4,
        # DIO pin numbers
        'LEFT_LIMIT_ID': 0,
        'RIGHT_LIMIT_ID': 1,
        'CABLE_WRAPPED': 'UNDER',
        # Both speeds positive.
        # Extend speed must be lower than natural extend rate
        'EXTEND_SPEED': 0.5,
        'RETRACT_SPEED': 0.7,
    },
}

gull_lake = {
    'CONTROLLERS': {
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
    },
    'DRIVETRAIN': {
        'LEFT': {
            'LEFT_LEADER_ID': 13,
            'LEFT_FOLLOWER_ID': 8,
        },
        'RIGHT': {
            'RIGHT_LEADER_ID': 15,
            'RIGHT_FOLLOWER_ID': 7,
        },
        'DRIVETYPE': ARCADE
    },
    'AIMER': {
        'AIMING_ROTATION_SPEED': 0.6,
        'AIMING_ACCURACY_DEGREES': 3,
    },
    'VISION': {
        'TARGET_HEIGHT': 8.5,
        'TARGET_RADIUS': 2,
        'SHOOTER_HEIGHT': 3.5,
        'SHOOTER_OFFSET': 1,
        'CAMERA_HEIGHT': 4,
        'CAMERA_PITCH': 0,
    },
    'TILTSHOOTER': {
        'TILTSHOOTER_ID': 1,
        'ROTATIONS_PER_360': 75,
        'MIN_DEGREES': 5,
        'MAX_DEGREES': 25,
        'BUFFER_DEGREES': 2,
        'SPEED': 0.1,
    },
    'SHOOTER': {
        'SHOOTER_ID': 10,
        'SHOOTER_RPM': 3500,
    },
    'FEEDER': {
      'FEEDER_ID' : 9,
      'FEEDER_SPEED': 0.4,
    },
    'CLIMBER': {
        'WINCH_LEFT_ID': 7,
        'WINCH_RIGHT_ID': 8,
        # Pneumatic board IDs
        'SOLENOID_FORWARD_ID': 3,
        'SOLENOID_REVERSE_ID': 4,
        # DIO pin numbers
        'LEFT_LIMIT_ID': 0,
        'RIGHT_LIMIT_ID': 1,
        'CABLE_WRAPPED': 'UNDER',
        # Both speeds positive.
        # Extend speed must be lower than natural extend rate
        'EXTEND_SPEED': 0.5,
        'RETRACT_SPEED': 0.7,
    },
}

practice = {
    'CONTROLLERS': {
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
    },
    'DRIVETRAIN': {
        'LEFT': {
            'LEFT_LEADER_ID': 13,
            'LEFT_FOLLOWER_ID': 3,
        },
        'RIGHT': {
            'RIGHT_LEADER_ID': 15,
            'RIGHT_FOLLOWER_ID': 7,
        },
        'DRIVETYPE': ARCADE
    },
    'AIMER': {
        'AIMING_ROTATION_SPEED': 0.6,
        'AIMING_ACCURACY_DEGREES': 3,
    },
    'VISION': {
        'TARGET_HEIGHT': 8.5,
        'TARGET_RADIUS': 2,
        'SHOOTER_HEIGHT': 3.5,
        'SHOOTER_OFFSET': 1,
        'CAMERA_HEIGHT': 4,
        'CAMERA_PITCH': 0,
    },
    'SHOOTER': {
        'SHOOTER_ID': 9,
        'SHOOTER_RPM': 1000,
    },
}

intakeTest = {
    'CONTROLLERS': {
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
    },
    'INTAKE': {
        'INTAKE_MOTOR_ID': 15,
        'INTAKE_SOLENOID_FORWARD_ID': 4,
        'INTAKE_SOLENOID_REVERSE_ID': 5
    }
}

climberTest = {
    'CONTROLLERS': {
        'OPERATOR': {
            'ID': 0,
            'DEADZONE': DEADZONE,
            'LEFT_TRIGGER_AXIS': 2,
            'RIGHT_TRIGGER_AXIS': 3,
        }
    },
    'CLIMBER': {
        'WINCH_LEFT_ID': 9,
        'WINCH_RIGHT_ID': 10,
        # Pneumatic board IDs
        'SOLENOID_FORWARD_ID': 1,
        'SOLENOID_REVERSE_ID': 0,
    },
}

visionTest = {
    'CONTROLLERS': {
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
    },
    'VISION': {
        'TARGET_HEIGHT': 8.5,
        'TARGET_RADIUS': 2,
        'SHOOTER_HEIGHT': 3.5,
        'SHOOTER_OFFSET': 1,
        'CAMERA_HEIGHT': 4,
        'CAMERA_PITCH': 0,
    },
    'AIMER': {
        'AIMING_ROTATION_SPEED': 0.6,
        'AIMING_ACCURACY_DEGREES': 3,
    },

    'DRIVETRAIN': {
        'LEFT': {
            'LEFT_LEADER_ID': 13,
            'LEFT_FOLLOWER_ID': 3,
        },
        'RIGHT': {
            'RIGHT_LEADER_ID': 14,
            'RIGHT_FOLLOWER_ID': 2,
        },
        'DRIVETYPE': ARCADE
    }
    # Robot IP is 10.10.76.2
}

autonTest = {
    'CONTROLLERS': {
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
    },
    'DRIVETRAIN': {
        'LEFT': {
            'LEFT_LEADER_ID': 13,
            'LEFT_FOLLOWER_ID': 3,
        },
        'RIGHT': {
            'RIGHT_LEADER_ID': 15,
            'RIGHT_FOLLOWER_ID': 2,
        },
        'DRIVETYPE': ARCADE
    },
    'SHOOTER': {
        'SHOOTER_ID': 9
    },
    'VISION': 'enabled',
}

robotconfig = gull_lake
