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

    def testTankDrive(self):
        # test tank drive
        self.robot.drive_type = robotconfig.TANK
        self.testDriverXBC.reset()
        self.testOperatorXBC.reset()
        self.testDriverXBC.responses['LEFT_Y'] = 0.9
        self.testDriverXBC.responses['RIGHT_Y'] = 0.9
        self.robot.teleopPeriodic()
        print('Tank Drive: Passed!')
        self.robot.drive_type = robotconfig.ARCADE
        

    def testArcadeDrive(self):
        # test arcade drive
        self.testDriverXBC.reset()
        self.testOperatorXBC.reset()
        self.testDriverXBC.responses['RIGHT_X'] = 0.9
        self.testDriverXBC.responses['RIGHT_Y'] = 0.9
        self.robot.teleopPeriodic()
        print('Arcade Drive: Passed!') # no assert() needed, just testing code doesn't break

    def testArcadeDriveWithAutoRotate(self):
        # test arcade drive
        self.testDriverXBC.reset()
        self.testOperatorXBC.reset()
        self.testDriverXBC.responses['RIGHT_BUMPER'] = True
        self.testDriverXBC.responses['LEFT_Y'] = 0.9
        self.testDriverXBC.responses['RIGHT_Y'] = 0.9
        self.robot.teleopPeriodic()
        print('Arcade Drive With Auto Rotate: Passed!')



