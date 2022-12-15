from email.policy import default
import robotconfig
#from aimer import Aimer

defaultResponses = {
    # CONTROLLER VALUES
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
    # AIMER VALUES
    'THETA': 0.0,
    'IN_RANGE': False,
}

"""
class TestAimer(Aimer):
    def __init__(self, aimer):
        super().__init__(aimer.gyro, aimer.rotationSpeed, aimer.accuracyDegrees)
        self.resetResponses()

    def resetResponses(self):
        self.responses = defaultResponses

    def getTheta(self):
        return self.responses['THETA']
"""


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

    def getPOV(self):
        return -1


class Tester():
    def __init__(self, robot):
        self.robot = robot
        #self.aimer = TestAimer(robot.aimer)
        #robot.aimer = self.aimer

    @staticmethod
    def getTestConfig():
        return robotconfig.testbot

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
