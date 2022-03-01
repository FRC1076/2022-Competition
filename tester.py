from pickle import TRUE
import robotconfig

defaultResponses = {
    'LEFT_Y': 0.0,
    'LEFT_X': 0.0,
    'RIGHT_Y': 0.0,
    'RIGHT_X': 0.0,
    'LEFT_BUMPER': False,
    'RIGHT_BUMPER': False,
    'RAW_AXIS': {
        2: 0.0,
        3: 0.0
    },
    'X_BUTTON': False,
    'Y_BUTTON': False,
    'A_BUTTON': False,
    'B_BUTTON': False,
}

class TestController():
    def __init__(self):
        self.xboxController = TestXBC()
        self.deadzone = 0.2
        self.left_trigger_axis = 2
        self.right_trigger_axis = 3

class TestXBC():
    def __init__(self):
        self.reset()

    def reset(self):
        self.responses = defaultResponses

    def getLeftY(self):
        return self.responses['LEFT_Y']
    
    def getLeftX(self):
        return self.responses['LEFT_X']
    
    def getRightY(self):
        return self.responses['RIGHT_Y']
    
    def getRightX(self):
        return self.responses['RIGHT_X']

    def getLeftBumper(self):
        return self.responses['LEFT_BUMPER']
    
    def getRightBumper(self):
        return self.responses['RIGHT_BUMPER']

    def getRawAxis(self, axis_id):
        return self.responses['RAW_AXIS'][axis_id]
    
    def getXButton(self):
        return self.responses['X_BUTTON']

    def getXButtonPressed(self):
        return self.responses['X_BUTTON']

    def getYButton(self):
        return self.responses['Y_BUTTON']

    def getYButtonPressed(self):
        return self.responses['Y_BUTTON']
    
    def getAButton(self):
        return self.responses['A_BUTTON']

    def getAButtonPressed(self):
        return self.responses['A_BUTTON']

    def getBButton(self):
        return self.responses['B_BUTTON']

    def getBButtonPressed(self):
        return self.responses['B_BUTTON']

class Tester():
    def __init__(self, robot):
        self.robot = robot

    def getTestConfig(self):
        return robotconfig.competition

    def initTestTeleop(self):
        self.testDriverController = TestController()
        self.testOperatorController = TestController()
        self.testDriverXBC = self.testDriverController.xboxController
        self.testOperatorXBC = self.testDriverController.xboxController
        self.robot.driver = self.testDriverController
        self.robot.operator = self.testOperatorController
        self.robot.teleopInit()

    def logResult(self, result):
        print(result)

    def testCodePaths(self):
        self.testTankDrive()
        self.testArcadeDrive()
        self.testArcadeDriveWithAutoRotate()
        self.testIntake()
        self.testManualShooter()
        self.testManualClimber()

    def testTankDrive(self):
        self.robot.drive_type = robotconfig.TANK
        self.testDriverXBC.reset()
        self.testOperatorXBC.reset()
        self.testDriverXBC.responses['LEFT_Y'] = 0.9
        self.testDriverXBC.responses['RIGHT_Y'] = 0.9
        self.robot.teleopPeriodic()
        self.robot.drive_type = robotconfig.ARCADE
        print('\n******************')
        print('Tank Drive: Passed!')
        print('******************\n')
        
    def testArcadeDrive(self):
        self.testDriverXBC.reset()
        self.testOperatorXBC.reset()
        self.testDriverXBC.responses['RIGHT_X'] = 0.9
        self.testDriverXBC.responses['RIGHT_Y'] = 0.9
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Arcade Drive: Passed!')
        print('******************\n')
        
    def testArcadeDriveWithAutoRotate(self):
        self.testDriverXBC.reset()
        self.testOperatorXBC.reset()
        self.testDriverXBC.responses['LEFT_BUMPER'] = True
        self.testDriverXBC.responses['RIGHT_BUMPER'] = True
        
        self.testDriverXBC.responses['LEFT_Y'] = 0.9
        self.testDriverXBC.responses['RIGHT_Y'] = 0.9
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Arcade Drive With Auto Rotate: Passed!')
        print('******************\n')
        
    def testIntake(self):
        self.testDriverXBC.reset()
        self.testOperatorXBC.reset()

        lta = self.testOperatorController.left_trigger_axis
        self.testOperatorXBC.responses['LEFT_BUMPER'] = True
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Intake Piston Toggle On: Passed!')
        print('******************\n')

        self.testOperatorXBC.responses['LEFT_BUMPER'] = True
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Intake Piston Toggle Off: Passed!')
        print('******************\n')

        self.testOperatorXBC.responses['LEFT_BUMPER'] = False

        self.testOperatorXBC.responses['RAW_AXIS'][lta] = 0.99
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Intake Motor Run: Passed!')
        print('******************\n')

        self.testOperatorXBC.responses['RAW_AXIS'][lta] = 0.5
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Intake Motor Stop: Passed!')
        print('******************\n')

    def testManualShooter(self):
        self.testDriverXBC.reset()
        self.testOperatorXBC.reset()

        rta = self.testOperatorController.right_trigger_axis
        self.testOperatorXBC.responses['RAW_AXIS'][rta] = 0.99
        self.testOperatorXBC.responses['X_BUTTON'] = False
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Full Speed Shooter: Passed!')
        print('******************\n')

        rta = self.testOperatorController.right_trigger_axis
        self.testOperatorXBC.responses['RAW_AXIS'][rta] = 0.99
        self.testOperatorXBC.responses['X_BUTTON'] = True
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Reduced Speed Shooter: Passed!')
        print('******************\n')

        rta = self.testOperatorController.right_trigger_axis
        self.testOperatorXBC.responses['RAW_AXIS'][rta] = 0.0
        self.testOperatorXBC.responses['X_BUTTON'] = True
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Reduced Speed Non-Shooter: Passed!')
        print('******************\n')

    def testManualClimber(self):
        self.testDriverXBC.reset()
        self.testOperatorXBC.reset()

        self.testOperatorXBC.responses['X_BUTTON'] = True
        self.robot.teleopPeriodic()
        self.testOperatorXBC.responses['X_BUTTON'] = True
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Manual Climber Piston Toggle: Passed!')
        print('******************\n')

        self.testOperatorXBC.responses['X_BUTTON'] = False
        self.testOperatorXBC.responses['LEFT_Y'] = 0.8
        self.robot.teleopPeriodic()
        print('\n******************')
        print('Manual Climber Winch: Passed!')
        print('******************\n')

