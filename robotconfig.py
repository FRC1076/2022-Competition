
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
      'LEFT_LEADER_ID': 1,
      'LEFT_MIDDLE_ID': 2,
      'LEFT_FOLLOWER_ID': 3,
    },
    'RIGHT': {
      'RIGHT_LEADER_ID': 4,
      'RIGHT_MIDDLE_ID': 5,
      'RIGHT_FOLLOWER_ID': 6,
    },
    'DRIVETYPE': ARCADE
  },
  'SHOOTER' : {
    'SHOOTER_ID': 1
  },
  'INTAKE' : 
  {
    # Update IDs when known
    'INTAKE_MOTOR_ID' : 8,
    'INTAKE_SOLENOID_FORWARD_ID' : 1,
    'INTAKE_SOLENOID_REVERSE_ID' : 2,
  },
  'CLIMBER': {
    'WINCH_LEFT_ID': 9,
    'WINCH_RIGHT_ID': 10,
    # Pneumatic board IDs
    'SOLENOID_LEFT_FORWARD_ID': 3,
    'SOLENOID_LEFT_REVERSE_ID': 4,
    'SOLENOID_RIGHT_FORWARD_ID': 5,  # 2
    'SOLENOID_RIGHT_REVERSE_ID': 6,  # 3
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
  'TILTSHOOTER' : {
    'TILTSHOOTER_ID': 7,
  },
  'AIMER': 'enabled'
}

intakeTest = {
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
  'INTAKE' : {
    'INTAKE_MOTOR_ID' : 15,
    'INTAKE_SOLENOID_FORWARD_ID' : 4,
    'INTAKE_SOLENOID_REVERSE_ID' : 5
  }
}

climberTest = {
  'CONTROLLERS' : {
    'OPERATOR': {
      'ID': 0,
      'DEADZONE': DEADZONE,
      'LEFT_TRIGGER_AXIS': 2,
      'RIGHT_TRIGGER_AXIS': 3,
    }
  },
  'CLIMBER' : {
    'WINCH_RIGHT_ID' : 15,
    'WINCH_LEFT_ID' : 15,
    'SOLENOID_RIGHT_FORWARD_ID' : 4,
    'SOLENOID_RIGHT_REVERSE_ID' : 5
  }
}

visionTest = {
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
  'VISION' : None
  # Robot IP is 10.10.76.2
}

robotconfig = practice
