import math
import time
import sys

import wpilib
import wpilib.drive
import wpimath.controller
from wpilib import interfaces
import rev
from navx import AHRS

from robotconfig import robotconfig
from controller import Controller
from swervedrive import SwerveDrive
from swervemodule import SwerveModule
from swervemodule import ModuleConfig
from feeder import Feeder
from tester import Tester
from networktables import NetworkTables

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

# Test Mode
TEST_MODE = False

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        print("robot init")

        self.drivetrain = None
        self.driver = None
        self.operator = None
        self.feeder = None
        self.tester = None
        self.auton = None

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

        flModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=2.97, inverted=True, allow_reverse=True)
        frModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=2.69, inverted=False, allow_reverse=True)
        rlModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=0.18, inverted=True, allow_reverse=True)
        rrModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=4.76, inverted=False, allow_reverse=True)

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushed

        # Drive motors
        flModule_driveMotor = rev.CANSparkMax(config['FRONTLEFT_DRIVEMOTOR'], motor_type)
        frModule_driveMotor = rev.CANSparkMax(config['FRONTRIGHT_DRIVEMOTOR'], motor_type)
        rlModule_driveMotor = rev.CANSparkMax(config['REARLEFT_DRIVEMOTOR'], motor_type)
        rrModule_driveMotor = rev.CANSparkMax(config['REARRIGHT_DRIVEMOTOR'], motor_type)

        # Rotate motors
        flModule_rotateMotor = rev.CANSparkMax(config['FRONTLEFT_ROTATEMOTOR'], motor_type)
        frModule_rotateMotor = rev.CANSparkMax(config['FRONTRIGHT_ROTATEMOTOR'], motor_type)
        rlModule_rotateMotor = rev.CANSparkMax(config['REARLEFT_ROTATEMOTOR'], motor_type)
        rrModule_rotateMotor = rev.CANSparkMax(config['REARRIGHT_ROTATEMOTOR'], motor_type)

        frontLeftModule = SwerveModule(flModule_driveMotor, flModule_rotateMotor, flModule_cfg)
        frontRightModule = SwerveModule(frModule_driveMotor, frModule_rotateMotor, frModule_cfg)
        rearLeftModule = SwerveModule(rlModule_driveMotor, rlModule_rotateMotor, rlModule_cfg)
        rearRightModule = SwerveModule(rrModule_driveMotor, rrModule_rotateMotor, rrModule_cfg)

        swerve = SwerveDrive(frontLeftModule, frontRightModule, rearLeftModule, rearRightModule)

        return swerve

    #EXAMPLE
    def initFeeder(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        feeder = rev.CANSparkMax(config['FEEDER_ID'], motor_type)
        return Feeder(feeder, config['FEEDER_SPEED'])


    def robotPeriodic(self):
        return True


    def teleopInit(self):
        return True


    def teleopPeriodic(self):
        print("teleopPeriodic!!!!!!!")
        self.teleopDrivetrain()
        return True

    def move(self, x, y, rcw):
        """
        This function is ment to be used by the teleOp.
        :param x: Velocity in x axis [-1, 1]
        :param y: Velocity in y axis [-1, 1]
        :param rcw: Velocity in z axis [-1, 1]
        """

        if self.driver.getLeftBumper():
            # If the button is pressed, lower the rotate speed.
            rcw *= 0.7

        self.drivetrain.move(x, y, rcw)
        self.drivetrain.execute()

    def teleopDrivetrain(self):
        print("teleopDrivetrain!!!!!!!")
        if (not self.drivetrain):
            print("teleopDrivetrain :((((")
            return

        driver = self.driver.xboxController
        deadzone = self.driver.deadzone

        # TANK DRIVE
        if (self.drive_type == TANK):
            speedratio = 1.0  # ratio of joystick position to motor speed

            # Get left and right joystick values.
            leftspeed = driver.getLeftY()
            rightspeed = -(driver.getRightY())

            # Eliminate deadzone and correct speed
            leftspeed = speedratio * self.deadzoneCorrection(leftspeed, deadzone)
            rightspeed = speedratio * self.deadzoneCorrection(rightspeed, deadzone)

            # Invoke Tank Drive
            self.drivetrain.tankDrive(leftspeed, rightspeed)

        # ARCADE DRIVE
        elif self.drive_type == ARCADE:
            # speedratio = 0.8  # ratio of joystick position to motor speed
            speedratio = 1.0

            result = (-driver.getRightX(), driver.getLeftY())

            rotateSpeed = result[0]
            driveSpeed = result[1]

            #print(rotateSpeed, driveSpeed)
            rotateSpeed = speedratio * self.deadzoneCorrection(rotateSpeed + self.rotationCorrection, deadzone)
            driveSpeed = speedratio * self.deadzoneCorrection(driveSpeed, deadzone)

            #print(rotateSpeed, driveSpeed)
            self.drivetrain.arcadeDrive(rotateSpeed, driveSpeed)

        else:  # self.drive == SWERVE
            # Drive
            self.move(driver.getRightX(), driver.getRightY(), driver.getLeftX())

            module = self.drivetrain.modules('front_left')

            print('DRIVE_POWER = ' + module.driveMotor.get())
            print('PIVOT_POWER = ' + module.rotateMotor.get())

            # Lock
            if self.gamempad.getRightBumper():
                self.drivetrain.request_wheel_lock = True

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
