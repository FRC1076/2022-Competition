import math
import time
import sys

import wpilib
import wpilib.drive
import wpimath.controller
from wpilib import interfaces
import rev
import ctre
from navx import AHRS

from robotconfig import robotconfig
from controller import Controller
from swervedrive import SwerveDrive
from swervemodule import SwerveModule
from swervemodule import ModuleConfig
from feeder import Feeder
from tester import Tester
from networktables import NetworkTables
from hooks import Hooks

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

# Test Mode
TEST_MODE = False

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        self.drivetrain = None
        self.driver = None
        self.operator = None
        self.feeder = None
        self.tester = None
        self.auton = None
        self.hooks = None

        # Even if no drivetrain, defaults to drive phase
        self.phase = "DRIVE_PHASE"

        if TEST_MODE:
            self.config = Tester.getTestConfig()
        else:
            self.config = robotconfig

        print(self.config)
        for key, config in self.config.items():
            if key == 'CONTROLLERS':
                controllers = self.initControllers(config)
                self.driver = controllers[0]
                self.operator = controllers[1]
            if key == 'DRIVETRAIN':
                self.drivetrain = self.initDrivetrain(config)
                print(self.drivetrain)
            if key == 'FEEDER':
                self.feeder = self.initFeeder(config)
            if key == 'AUTON':
                self.auton = self.initAuton(config)
            if key == 'HOOKS':
                self.hooks = self.initHooks(config)

        self.dashboard = NetworkTables.getTable('SmartDashboard')
        self.periods = 0

        if TEST_MODE:
            self.tester = Tester(self)
            self.tester.initTestTeleop()
            self.tester.testCodePaths()
            

    def initControllers(self, config):
        ctrls = {}
        print(config)
        for ctrlConfig in config.values():
            print(ctrlConfig)
            controller_id = ctrlConfig['ID']
            ctrl = wpilib.XboxController(controller_id)
            dz = ctrlConfig['DEADZONE']
            lta = ctrlConfig['LEFT_TRIGGER_AXIS']
            rta = ctrlConfig['RIGHT_TRIGGER_AXIS']
            ctrls[controller_id] = Controller(ctrl, dz, lta, rta)
        return ctrls


    def initAuton(self, config):
        return True


    def initDrivetrain(self, config):
        
        self.drive_type = config['DRIVETYPE']  # side effect!

        self.rotationCorrection = config['ROTATION_CORRECTION']

        flModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=0.0, inverted=True, allow_reverse=True)
        frModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=0.0, inverted=False, allow_reverse=True)
        rlModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=0.0, inverted=True, allow_reverse=True)
        rrModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=0.0, inverted=False, allow_reverse=True)

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless

        # Drive motors
        #flModule_driveMotor = rev.CANSparkMax(config['FRONTLEFT_DRIVEMOTOR'], motor_type)
        #frModule_driveMotor = rev.CANSparkMax(config['FRONTRIGHT_DRIVEMOTOR'], motor_type)
        #rlModule_driveMotor = rev.CANSparkMax(config['REARLEFT_DRIVEMOTOR'], motor_type)
        #rrModule_driveMotor = rev.CANSparkMax(config['REARRIGHT_DRIVEMOTOR'], motor_type)

        # Rotate motors
        #flModule_rotateMotor = rev.CANSparkMax(config['FRONTLEFT_ROTATEMOTOR'], motor_type)
        #frModule_rotateMotor = rev.CANSparkMax(config['FRONTRIGHT_ROTATEMOTOR'], motor_type)
        #rlModule_rotateMotor = rev.CANSparkMax(config['REARLEFT_ROTATEMOTOR'], motor_type)
        #rrModule_rotateMotor = rev.CANSparkMax(config['REARRIGHT_ROTATEMOTOR'], motor_type)

        #flModule_encoder = ctre.CANCoder(config['FRONTLEFT_ENCODER'])
        #frModule_encoder = ctre.CANCoder(config['FRONTRIGHT_ENCODER'])
        #rlModule_encoder = ctre.CANCoder(config['REARLEFT_ENCODER'])
        #rrModule_encoder = ctre.CANCoder(config['REARRIGHT_ENCODER'])

        #frontLeftModule = SwerveModule(flModule_driveMotor, flModule_rotateMotor, flModule_encoder, flModule_cfg)
        #frontRightModule = SwerveModule(frModule_driveMotor, frModule_rotateMotor, frModule_cfg)
        #rearLeftModule = SwerveModule(rlModule_driveMotor, rlModule_rotateMotor, rlModule_cfg)
        #rearRightModule = SwerveModule(rrModule_driveMotor, rrModule_rotateMotor, rrModule_cfg)

        #swerve = SwerveDrive(frontLeftModule, frontRightModule, rearLeftModule, rearRightModule)

        #return swerve

        #self.testingModule = frontLeftModule

    #EXAMPLE
    def initFeeder(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        feeder = rev.CANSparkMax(config['FEEDER_ID'], motor_type)
        return Feeder(feeder, config['FEEDER_SPEED'])


    def robotPeriodic(self):
        return True


    def teleopInit(self):
        print("teleopInit ran")
        return True


    def teleopPeriodic(self):
        self.teleopDrivetrain()
        self.te
        return True

    def move(self, x, y, rcw):
        """
        This function is ment to be used by the teleOp.
        :param x: Velocity in x axis [-1, 1]
        :param y: Velocity in y axis [-1, 1]
        :param rcw: Velocity in z axis [-1, 1]
        """

        # if self.driver.getLeftBumper():
        #     # If the button is pressed, lower the rotate speed.
        #     rcw *= 0.7

        degrees = (math.atan2(y, x) * 180 / math.pi) + 180

        self.testingModule.move(rcw, degrees)
        self.testingModule.execute()

        print('DRIVE_TARGET = ' + str(rcw) + ', PIVOT_TARGET = ' + str(degrees) + ", ENCODER_TICK = " + str(self.testingModule.get_current_angle()))
        print('DRIVE_POWER = ' + str(self.testingModule.driveMotor.get()) + ', PIVOT_POWER = ' + str(self.testingModule.rotateMotor.get()))

        #self.drivetrain.move(x, y, rcw)
        #self.drivetrain.execute()

    def teleopDrivetrain(self):
        # if (not self.drivetrain):
        #     return

        driver = self.driver.xboxController
        deadzone = self.driver.deadzone

        #self.move(driver.getRightX(), driver.getRightY(), driver.getLeftY())

        #self.testingModule.testMove(driver.getLeftY(), driver.getRightX())

        # Vectoral Button Drive
        #if self.gamempad.getPOV() == 0:
        #    self.drive.set_raw_fwd(-0.35)
        #elif self.gamempad.getPOV() == 180:
        #    self.drive.set_raw_fwd(0.35)
        #elif self.gamempad.getPOV() == 90:
        #    self.drive.set_raw_strafe(0.35)
        #elif self.gamempad.getPOV() == 270:
        #    self.drive.set_raw_strafe(-0.35)
        return

    def initHooks(self, config):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushed

        #Front
        hook1 = rev.CANSparkMax(config['FRONT_HOOK_ID'], motor_type)

        #Back
        hook2 = rev.CANSparkMax(config['BACK_HOOK_ID'], motor_type)

        #Left
        hook3 = rev.CANSparkMax(config['LEFT_HOOK_ID'], motor_type)

        #Right
        hook4 = rev.CANSparkMax(config['RIGHT_HOOK_ID'], motor_type)

        return Hooks([hook1, hook2, hook3, hook4])
    
    def teleopHooks(self):
        operator = self.operator.xboxController

        if operator.xboxController.getYButtonReleased():
            self.hooks.change_front()
        
        if operator.xboxController.getAButtonReleased():
            self.hooks.change_back()
        
        if operator.xboxController.getXButtonReleased():
            self.hooks.change_left()
        
        if operator.xboxController.getBButtonReleased():
            self.hooks.change_right()


    def autonomousInit(self):
        if not self.auton:
            return


    def autonomousPeriodic(self):
        if not self.auton:
            return

        self.autonForwardAndBack()


    def autonForwardAndBack(self):
        driver = self.driver.xboxController
        if driver.getLeftBumper() and driver.getRightBumper():
            if self.autonTimer.get() < 1.0:
                self.drivetrain.arcadeDrive(0, -0.75)
            elif 1.0 <= self.autonTimer.get() < 2.0:
                self.drivetrain.arcadeDrive(0, 0.75)
        

    def deadzoneCorrection(self, val, deadzone):
        """
        Given the deadzone value x, the deadzone both eliminates all
        values between -x and x, and scales the remaining values from
        -1 to 1, to (-1 + x) to (1 - x)
        """
        if abs(val) < deadzone:
            return 0
        elif val < 0:
            x = (abs(val) - deadzone) / (1 - deadzone)
            return -x
        else:
            x = (val - deadzone) / (1 - deadzone)
            return x

    def logResult(self, *result):
        if (TEST_MODE):
            print(result)


if __name__ == "__main__":
    #if sys.argv[1] == 'sim':
    #    TEST_MODE = True
    wpilib.run(MyRobot)
