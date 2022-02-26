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

testDriverController = TestController()
testOperatorController = TestController()
testDriverXBC = testDriverController.xboxController
testOperatorXBC = testDriverController.xboxController

def initTestConfig(robot):
    robot.config = robotconfig.competition

def initTestControllers(robot):
    robot.driver = testDriverController
    robot.operator = testOperatorController

def initTestTeleop(robot):
    robot.teleopInit()

def testCodePaths(robot):
    #TODO: test tank drive

    # test arcade drive
    testDriverXBC.reset()
    testOperatorXBC.reset()

    testDriverXBC.responses['RIGHT_X'] = 0.9
    testDriverXBC.responses['RIGHT_Y'] = 0.9

    robot.teleopPeriodic()
    print('Arcade Drive: Passed!') # no assert() needed, just testing code doesn't break

    #TODO: test arcade drive with auto-rotate

    #TODO: test intake

    #TODO: test manual shooter (ignore auto-shooter)

    #TODO: test tilt shooter

    #TODO: test climber

    #TODO: test intake

    #TODO: other tests as needed...





