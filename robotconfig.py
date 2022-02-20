
DEADZONE = 0.2

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

competition = {
  'CONTROLLERS' : {
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
  'DRIVETRAIN' : {
    'LEFT': {
      'LEFT_LEADER_ID': 13,
      'LEFT_MIDDLE_ID': 1,
      'LEFT_FOLLOWER_ID': 3,
    },
    'RIGHT': {
      'RIGHT_LEADER_ID': 14,
      'RIGHT_MIDDLE_ID': 15,
      'RIGHT_FOLLOWER_ID': 2,
    },
    'DRIVETYPE': ARCADE
  },
  'SHOOTER' : {
    'SHOOTER_ID': 1
  },
  'CLIMBER': {
    'WINCH_LEFT_ID': 1,
    'WINCH_RIGHT_ID': 15,
    # Pneumatic board IDs
    'SOLENOID_LEFT_FORWARD_ID': 0,
    'SOLENOID_LEFT_REVERSE_ID': 1,
    'SOLENOID_RIGHT_FORWARD_ID': 4,  # 2
    'SOLENOID_RIGHT_REVERSE_ID': 5,  # 3
  },
  'AIMER': 'enabled',
  'VISION': 'enabled'
}

practice = {
  'CONTROLLERS' : {
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
  'DRIVETRAIN' : {
    'LEFT': {
      'LEFT_LEADER_ID': 13,
      'LEFT_FOLLOWER_ID': 3,
    },
    'RIGHT': {
      'RIGHT_LEADER_ID': 14,
      'RIGHT_FOLLOWER_ID': 2,
    },
    'DRIVETYPE': ARCADE
  },
  'SHOOTER' : {
    'SHOOTER_ID': 1
  },
  'AIMER': 'enabled'
}

robotconfig = practice
