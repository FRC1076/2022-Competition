import math
import time
import sys

import wpilib
import wpilib.drive
import wpimath.controller
from wpilib import interfaces
import rev
from navx import AHRS
from intake import Intake

from robotconfig import robotconfig, autoAimTable
import climber  # not needed?
from climber import Climber, WinchGroup
from vision import Vision
from drivetrain import Drivetrain
from aimer import Aimer
from shooter import Shooter
from tiltshooter import TiltShooter
from feeder import Feeder
from controller import Controller
from tester import Tester
from networktables import NetworkTables

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

# Dashboard keys
PARAM_SHOOTER_RPM = 'PARAM_SHOOTER_RPM'
PARAM_TILT_ANGLE = 'PARAM_TILT_ANGLE'
PARAM_CLIMB_EXTEND = 'PARAM_CLIMB_EXTEND'
PARAM_CLIMB_RETRACT = 'PARAM_CLIMB_RETRACT'

VALUE_PHASE = 'VALUE_PHASE'
VALUE_SHOOTER_RPM = 'VALUE_SHOOTER_RPM'
VALUE_TILT_ANGLE = 'VALUE_TILT_ANGLE'
VALUE_VISION_HAS_TARGET = 'VALUE_VISION_HAS_TARGET'
VALUE_VISION_NUM_TARGETS = 'VALUE_VISION_NUM_TARGETS'
VALUE_VISION_PITCH = 'VALUE_VISION_PITCH'
VALUE_VISION_YAW = 'VALUE_VISION_YAW'
VALUE_VISION_DISTANCE = 'VALUE_VISION_DISTANCE'
VALUE_GYRO_RELATIVE_YAW = 'VALUE_GYRO_RELATIVE_YAW'
VALUE_GYRO_ACCUMULATED_YAW = 'VALUE_GYRO_ACCUMULATED_YAW'
VALUE_SHOT_FIRED = 'VALUE_SHOT_FIRED'

# Test Mode
TEST_MODE = False

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        self.drivetrain = None
        self.driver = None
        self.operator = None
        self.shooter = None
        self.tiltShooter = None
        self.feeder = None
        self.intake = None
        self.climber = None
        self.aimer = None
        self.vision = None
        self.tester = None
        self.auton = None

        # Even if no drivetrain, defaults to drive phase
        self.phase = "DRIVE_PHASE"

        if TEST_MODE:
            self.config = Tester.getTestConfig()
        else:
            self.config = robotconfig

        for key, config in self.config.items():
            if key == 'CONTROLLERS':
                controllers = self.initControllers(config)
                self.driver = controllers[0]
                self.operator = controllers[1]
            if key == 'DRIVETRAIN':
                self.drivetrain = self.initDrivetrain(config)
            if key == 'FEEDER':
                self.feeder = self.initFeeder(config)
            if key == 'SHOOTER':
                self.shooter = self.initShooter(config)
            if key == 'TILTSHOOTER':
                self.tiltShooter = self.initTiltShooter(config)
            if key == 'INTAKE':
                self.intake = self.initIntake(config)
            if key == 'CLIMBER':
                self.climber = self.initClimber(config)
            if key == 'AIMER':
                self.aimer = self.initAimer(config)
            if key == 'VISION':
                self.vision = self.initVision(config)
            if key == 'AUTON':
                self.auton = self.initAuton(config)

        self.dashboard = NetworkTables.getTable('SmartDashboard')
        self.initDashboard()

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
        self.autonTilting1Time = config['TILTING_1_TIME']
        self.autonSpinUp1Time = config['SPINUP_1_TIME']
        self.autonFiring1Time = config['FIRING_1_TIME']
        self.autonDrive1Time = config['DRIVE_1_TIME']
        self.autonRotate1Time = config['ROTATE_1_TIME']
        self.autonIntakeTime = config['INTAKE_TIME']
        self.autonRotate2Time = config['ROTATE_2_TIME']
        self.autonDrive2Time = config['DRIVE_2_TIME']
        self.autonTilting2Time = config['TILTING_2_TIME']
        self.autonSpinUp2Time = config['SPINUP_2_TIME']
        self.autonFiring2Time = config['FIRING_2_TIME']

        self.autonShootSpeed = config['SHOOT_SPEED']
        self.autonRotateSpeed = config['ROTATE_SPEED']
        self.autonDriveSpeed = config['DRIVE_SPEED']

        self.autonTilt1TargetDegrees = config['TILT_1_TARGET_DEGREES']

        if(config['POSITION'] == 1):
            self.autonTilt2TargetDegrees = config['POS_1_TILT_2_TARGET_DEGREES']
            self.autonRotate1TargetDegrees = config['POS_1_ROTATE_1_TARGET_DEGREES']
            self.autonRotate2TargetDegrees = config['POS_1_ROTATE_2_TARGET_DEGREES']
            self.autonDrive1Distance = config['POS_1_DRIVE_1_DISTANCE']
            self.autonDrive2Distance = config['POS_1_DRIVE_2_DISTANCE']

        elif(config['POSITION'] == 2):
            self.autonTilt2TargetDegrees = config['POS_2_TILT_2_TARGET_DEGREES']
            self.autonRotate1TargetDegrees = config['POS_2_ROTATE_1_TARGET_DEGREES']
            self.autonRotate2TargetDegrees = config['POS_2_ROTATE_2_TARGET_DEGREES']
            self.autonDrive1Distance = config['POS_2_DRIVE_1_DISTANCE']
            self.autonDrive2Distance = config['POS_2_DRIVE_2_DISTANCE']
        
        else: #  (config['POSITION'] == 3)
            self.autonTilt2TargetDegrees = config['POS_3_TILT_2_TARGET_DEGREES']
            self.autonRotate1TargetDegrees = config['POS_3_ROTATE_1_TARGET_DEGREES']
            self.autonRotate2TargetDegrees = config['POS_3_ROTATE_2_TARGET_DEGREES']
            self.autonDrive1Distance = config['POS_3_DRIVE_1_DISTANCE']
            self.autonDrive2Distance = config['POS_3_DRIVE_2_DISTANCE']

        return True

    def initDrivetrain(self, config):
        left_motors = []
        right_motors = []
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushed
        left_encoder_motor = None
        right_encoder_motor = None

        for controller_id in config['LEFT'].values():
            motor = rev.CANSparkMax(controller_id, motor_type)
            #motor.setOpenLoopRampRate(config['OPEN_LOOP_RAMP_RATE'])
            left_motors.append(motor)
            if 'LEFT_ENCODER' in config.keys() and controller_id == config['LEFT_ENCODER']:
                left_encoder_motor = motor

        for controller_id in config['RIGHT'].values():
            motor = rev.CANSparkMax(controller_id, motor_type)
            #motor.setOpenLoopRampRate(config['OPEN_LOOP_RAMP_RATE'])
            right_motors.append(motor)
            if 'RIGHT_ENCODER' in config.keys() and controller_id == config['RIGHT_ENCODER']:
                right_encoder_motor = motor

        self.drive_type = config['DRIVETYPE']  # side effect!
        self.clutchMultiplier = config['CLUTCH_MULTIPLIER']
        self.rotationCorrection = config['ROTATION_CORRECTION']

        # Create Controller Groups
        left_side = wpilib.MotorControllerGroup(*left_motors)
        right_side = wpilib.MotorControllerGroup(*right_motors)

        # Create Drivetrain
        motors = wpilib.drive.DifferentialDrive(left_side, right_side)
        return Drivetrain(motors, config['GEAR_RATIO'], config['WHEEL_CIRCUMFERENCE'], config['COUNTS_PER_REVOLUTION'], left_encoder_motor, right_encoder_motor)

    def initShooter(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        shooter_motor = rev.CANSparkMax(config['SHOOTER_ID'], motor_type)
        shooter = Shooter(shooter_motor, config['SHOOTER_RPM'], config['SHOOTER_MAX_RPM'], config['SHOOTER_MIN_RPM'])
        shooter.setPIDController()
        shooter.pidController.setFF(0.000167)  # assuming max shooter RPM of 6000
        return shooter

    def initFeeder(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        feeder = rev.CANSparkMax(config['FEEDER_ID'], motor_type)
        return Feeder(feeder, config['FEEDER_SPEED'])

    def initTiltShooter(self, config):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        tiltShooter = rev.CANSparkMax(config['TILTSHOOTER_ID'], motor_type)
        tiltShooter = TiltShooter(tiltShooter, config['ROTATIONS_PER_360'], config['MIN_DEGREES'],
                                  config['MAX_DEGREES'], config['BUFFER_DEGREES'], config['SPEED'])
        return tiltShooter

    def initIntake(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        pneumatics_module_type = wpilib.PneumaticsModuleType.CTREPCM
        motor = rev.CANSparkMax(config['INTAKE_MOTOR_ID'], motor_type)
        solenoid = wpilib.DoubleSolenoid(0, pneumatics_module_type, config['INTAKE_SOLENOID_FORWARD_ID'], config['INTAKE_SOLENOID_REVERSE_ID'])

        return Intake(solenoid, motor, config['INTAKE_MOTOR_SPEED'])

    def initClimber(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        winch_motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        pneumatics_module_type = wpilib.PneumaticsModuleType.CTREPCM

        right_winch = rev.CANSparkMax(config['WINCH_RIGHT_ID'], winch_motor_type)
        left_winch = rev.CANSparkMax(config['WINCH_LEFT_ID'], winch_motor_type)
        # winch = wpilib.MotorControllerGroup(right_winch, left_winch)

        # right_piston = wpilib.DoubleSolenoid(0,
        #    pneumatics_module_type, 
        #    config['SOLENOID_RIGHT_FORWARD_ID'], 
        #    config['SOLENOID_RIGHT_REVERSE_ID'])
        # left_piston = wpilib.DoubleSolenoid(0,
        #     pneumatics_module_type, 
        #     config['SOLENOID_LEFT_FORWARD_ID'], 
        #     config['SOLENOID_LEFT_REVERSE_ID'])

        # piston = SolenoidGroup([right_piston, left_piston])

        piston = wpilib.DoubleSolenoid(0,
                                       pneumatics_module_type,
                                       config['SOLENOID_FORWARD_ID'],
                                       config['SOLENOID_REVERSE_ID'])
        print(config)
        right_limit = wpilib.DigitalInput(config['RIGHT_LIMIT_ID'])
        left_limit = wpilib.DigitalInput(config['LEFT_LIMIT_ID'])

        winch = WinchGroup(right_winch, left_winch, right_limit, left_limit, config['CABLE_WRAPPED'])

        return Climber(piston, winch, config['EXTEND_SPEED'], config['RETRACT_SPEED'])

    def initAimer(self, config):
        ahrs = AHRS.create_spi()
        # navx = navx.AHRS.create_i2c()
        aimer = Aimer(ahrs, config['AIMING_ROTATION_SPEED'], config['AIMING_ACCURACY_DEGREES'])
        # aimer = Aimer(ahrs, 0.6, 3)
        aimer.reset()

        return aimer

    def initVision(self, config):
        vision = Vision(config['TARGET_HEIGHT'], config['TARGET_RADIUS'], config['SHOOTER_HEIGHT'],
                        config['SHOOTER_OFFSET'], config['CAMERA_HEIGHT'], config['CAMERA_PITCH'])
        # NetworkTables.initialize(server="10.10.76.2")
        # self.vision_table = NetworkTables.getTable("photonvision/mmal_service_16.1")
        return vision

        
    def initDashboard(self):

        if self.shooter:
            self.dashboard.putNumber(PARAM_SHOOTER_RPM, self.shooter.getConfigShooterRPM())
        
        if self.tiltShooter:
            self.dashboard.putNumber(PARAM_TILT_ANGLE, self.tiltShooter.getTargetDegrees())
        
        if self.climber:
            self.dashboard.putNumber(PARAM_CLIMB_EXTEND, self.climber.getExtendSpeed())
            self.dashboard.putNumber(PARAM_CLIMB_RETRACT, self.climber.getRetractSpeed())

        # make sure all VALUE fields exist
        self.dashboard.putString(VALUE_PHASE, 'INIT')
        self.dashboard.putNumber(VALUE_SHOOTER_RPM, 0)
        self.dashboard.putNumber(VALUE_TILT_ANGLE, 0)
        self.dashboard.putBoolean(VALUE_VISION_HAS_TARGET, False)
        self.dashboard.putNumber(VALUE_VISION_NUM_TARGETS, 0)
        self.dashboard.putNumber(VALUE_VISION_PITCH, 0)
        self.dashboard.putNumber(VALUE_VISION_YAW, 0)
        self.dashboard.putNumber(VALUE_VISION_DISTANCE, 0)
        self.dashboard.putNumber(VALUE_GYRO_RELATIVE_YAW, 0)
        self.dashboard.putNumber(VALUE_GYRO_ACCUMULATED_YAW, 0)
        self.dashboard.putBoolean(VALUE_SHOT_FIRED, False)

    def readDashboardParams(self):

        if self.shooter:
            newShooterRPM = self.dashboard.getNumber(PARAM_SHOOTER_RPM, -1.0)
            if newShooterRPM != -1.0:
                self.shooter.setStoredShooterRPM(newShooterRPM) # will clip to min/max
                self.dashboard.putNumber(PARAM_SHOOTER_RPM, self.shooter.getStoredShooterRPM())

        if self.tiltShooter:
            newTiltAngle = self.dashboard.getNumber(PARAM_TILT_ANGLE, -1.0)
            if newTiltAngle != -1.0:
                self.tiltShooter.setTargetDegrees(newTiltAngle) # will clip to min/max
                self.dashboard.putNumber(PARAM_TILT_ANGLE, self.tiltShooter.getTargetDegrees())
        
        if self.climber:
            newClimberExtend = self.dashboard.getNumber(PARAM_CLIMB_EXTEND, -1.0)
            if newClimberExtend != -1.0:
                self.climber.setExtendSpeed(newClimberExtend)
                self.dashboard.putNumber(PARAM_CLIMB_EXTEND, self.climber.getExtendSpeed())

            newClimberRetract = self.dashboard.getNumber(PARAM_CLIMB_RETRACT, -1.0)
            if newClimberRetract != -1.0:
                self.climber.setRetractSpeed(newClimberRetract)
                self.dashboard.putNumber(PARAM_CLIMB_RETRACT, self.climber.getRetractSpeed())


    
    def updateDashboardInfo(self):

        self.dashboard.putString(VALUE_PHASE, self.phase)

        if (self.vision):
            self.dashboard.putBoolean(VALUE_VISION_HAS_TARGET, self.vision.hasTargets())
            if (self.vision.hasTargets()):
                self.dashboard.putNumber(VALUE_VISION_YAW, self.vision.getSmoothYaw())
                self.dashboard.putNumber(VALUE_VISION_PITCH, self.vision.getSmoothPitch())
                self.dashboard.putNumber(VALUE_VISION_DISTANCE, self.vision.getDistanceFeet())
                self.dashboard.putNumber(VALUE_VISION_NUM_TARGETS, self.vision.getNumTargets())

        if (self.aimer):
            # self.dashboard.putNumber('Aimer In Range (T/F)', self.aimer.getInRange())
            self.dashboard.putNumber(VALUE_GYRO_RELATIVE_YAW, self.aimer.getYaw())
            self.dashboard.putNumber(VALUE_GYRO_ACCUMULATED_YAW, self.aimer.getAccumulatedYaw())

        if (self.tiltShooter):
            self.dashboard.putNumber(VALUE_TILT_ANGLE, self.tiltShooter.getDegrees())

        if (self.shooter):
            self.dashboard.putNumber(VALUE_SHOOTER_RPM, self.shooter.encoder.getVelocity())

        if (self.feeder):
            self.dashboard.putBoolean(VALUE_SHOT_FIRED, self.feeder.hasFired())


    def robotPeriodic(self):
        self.readDashboardParams()
        self.updateDashboardInfo()


    def teleopInit(self):
        self.theta = None

        # Even if no drivetrain, defaults to drive phase
        self.phase = "DRIVE_PHASE"

        if self.climber:  # false if no climber initialized
            # Climber presets
            self.climbRunning = False
            self.t = time.time()

            self.tm = wpilib.Timer()
            self.tm.start()

            self.climber.solenoids.set(climber.kReverse)

    def teleopPeriodic(self):

        driver = self.driver.xboxController
        rta = self.driver.right_trigger_axis

        if (self.vision is None or self.aimer is None or self.tiltShooter is None):
            self.phase = "DRIVE_PHASE"

        if (self.phase == "DRIVE_PHASE"):
            #print("TELEOP: DRIVE_PHASE")
            self.teleopVision()
            self.teleopDrivetrain()
            self.teleopIntake()
            self.teleopTiltShooter()
            # self.teleopShooter()
            self.teleopShooter(shooterRPM=self.shooter.getConfigShooterRPM())
            self.teleopFeeder()
            self.teleopClimber()
        elif (self.phase == "AS_ROTATE_PHASE"):
            print("TELEOP: AS_ROTATE_PHASE")
            if (driver.getRawAxis(rta) > 0.95):
                self.phase = "DRIVE_PHASE"
                self.teleopDrivetrain()
            else:
                if self.aimer.isInRange():
                    self.phase = "AS_TILT_PHASE"
                    currentDistance = self.vision.getDistanceFeet()
                    autoAimResult = self.autoAimLookup(currentDistance)
                    autoAimRPM = autoAimResult[0]
                    autoAimTilt = autoAimResult[1]
                    self.tiltShooter.setTargetDegrees(autoAimTilt)
                    self.shooter.setStoredShooterRPM(autoAimRPM)
                    self.teleopTiltShooter()
                    self.teleopShooter(shooterRPM=self.shooter.getStoredShooterRPM())
                    print("Entering AS_TILT_PHASE: RPM: ", autoAimRPM, " tilt degrees: ", autoAimTilt)
                else:
                    print("Still rotating ...")
                    self.teleopDrivetrain()
                self.teleopShooter(shooterRPM=self.shooter.getStoredShooterRPM())
        elif (self.phase == "AS_TILT_PHASE"):
            #print("TELEOP: AS_TILT_PHASE")
            if (driver.getRawAxis(rta) > 0.95):
                self.phase = "DRIVE_PHASE"
                self.teleopDrivetrain()
            else:
                if (self.tiltShooter.isNearTarget()):
                    self.phase = "AS_FIRE_PHASE"
                    print("Entering AS_FIRE_PHASE")
                else:
                    self.teleopTiltShooter()
                self.teleopShooter(shooterRPM=self.shooter.getStoredShooterRPM())
        elif (self.phase == "AS_FIRE_PHASE"):
            #print("TELEOP: AS_FIRE_PHASE)")
            if (driver.getRawAxis(rta) > 0.95):
                self.phase = "DRIVE_PHASE"
                self.teleopDrivetrain()
            else:
                self.teleopShooter(shooterRPM=self.shooter.getStoredShooterRPM())
                self.teleopFeeder()
                if (self.feeder.hasFired()):
                    print("Entering DRIVE_PHASE")
                    self.phase = "DRIVE_PHASE"
                    self.feeder.reset()

    def teleopVision(self):
        if (not self.vision):
            return

        targets = self.vision.getLatestResult()
        # yaw = self.vision_table.getNumber("targetPitch", )
        # print(self.vision.getYawDegrees(), self.vision.getSmoothYaw(), self.vision.getDistanceFeet())

    def teleopDrivetrain(self):
        if (not self.drivetrain):
            return

        driver = self.driver.xboxController
        lta = self.driver.left_trigger_axis
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
            self.drivetrain.motors.tankDrive(leftspeed, rightspeed)

        # ARCADE DRIVE
        elif self.drive_type == ARCADE:
            # speedratio = 0.8  # ratio of joystick position to motor speed
            speedratio = 1.0

            result = (-driver.getRightX(), driver.getLeftY())

            if (self.vision and self.aimer):

                if (self.phase == "DRIVE_PHASE"):

                    if driver.getLeftBumper():  # for testing auto-rotate
                        self.aimer.reset()

                    if (driver.getRightBumperPressed() and self.vision.getSmoothYaw()):

                        # transition phase if bumper is pressed and we have targets
                        self.aimer.setError(self.vision.getSmoothYaw())
                        self.aimer.setGyroSetPoint(self.aimer.getAccumulatedYaw() + self.aimer.getError())
                        self.phase = "AS_ROTATE_PHASE"

                elif (self.phase == "AS_ROTATE_PHASE"): 

                    # Want to 0 out the diff, which is either the current yaw from vision
                    if (self.vision.getSmoothYaw()):
                        self.aimer.setError(self.vision.getSmoothYaw())
                        self.aimer.setGyroSetPoint(self.aimer.getAccumulatedYaw() + self.aimer.getError())
                    elif (self.aimer.getGyroSetPoint() is not None):
                        self.aimer.setError(self.aimer.getGyroSetPoint() - self.aimer.getAccumulatedYaw())
                    else:
                        print("should never happen")
                        self.phase = "DRIVE_PHASE"
                    
                    result = (self.aimer.PIDcalc(self.aimer.getError()), 0)

                else:
                    print("In AS_AIMER_PHASE: should never happen")
                    self.phase = "DRIVE_PHASE"

            rotateSpeed = result[0]
            driveSpeed = result[1]

            #print("rotateSpeed: ", rotateSpeed, " drivespeed ", driveSpeed)

            #rotateSpeed = speedratio * self.deadzoneCorrection(rotateSpeed + self.rotationCorrection, deadzone)
            #driveSpeed = speedratio * self.deadzoneCorrection(driveSpeed, deadzone)

            #print("After Correction: rotateSpeed: ", rotateSpeed, " drivespeed ", driveSpeed)

            #print(rotateSpeed, driveSpeed)

            clutchMultiplier = 1.0
            if(driver.getRawAxis(lta) > 0.95):
                clutchMultiplier = self.clutchMultiplier
            self.drivetrain.motors.arcadeDrive(rotateSpeed * clutchMultiplier, driveSpeed * clutchMultiplier)

        else:  # self.drive == SWERVE
            # PANIC
            return

        #print("rotations (L/R): ", self.drivetrain.getLeftInches(), self.drivetrain.getRightInches())

    def teleopIntake(self):
        '''
        Manually controls the intake solenoid and motor.
        The left bumper toggles the position of the solenoid,
        and the left trigger turns the motor on when pressed
        in fully.
        '''
        if self.intake is None:
            return

        operator = self.operator.xboxController
        lta = self.operator.left_trigger_axis

        if operator.getLeftBumper():
            self.intake.toggle()
        
        if operator.getRawAxis(lta) > 0.95:
            self.intake.motorOn()
        else:
            self.intake.motorOff()
        

    def teleopTiltShooter(self):
        if not self.tiltShooter:
            return

        speed = self.tiltShooter.getTargetSpeed()
        # buffer = self.tiltShooter.getBufferDegrees()
        lta = self.operator.left_trigger_axis
        operator = self.operator.xboxController

        if (self.phase == "AS_TILT_PHASE"):  # Adjusting tiltShooter automatically with targetDegrees
            self.tiltShooterPeriodic()

        else:  # Adjusting tiltShooter mannual
            #print("TELEOP TILT SHOOTER MANUAL: getPoV", operator.getPOV())
            if (operator.getXButton()):
                self.tiltShooter.resetPosition()
            elif (operator.getYButton()) and (self.getPOVRange(operator.getPOV()) == 180):
                self.tiltShooter.setSpeed(-speed)
                #print("negative speed")
            elif (self.getPOVRange(operator.getPOV()) == 180) and (self.tiltShooter.getDegrees() > self.tiltShooter.getMinDegrees()):
                self.tiltShooter.setSpeed(-speed)
                #print("negative speed")
            elif (self.getPOVRange(operator.getPOV()) == 0) and (self.tiltShooter.getDegrees() < self.tiltShooter.getMaxDegrees()):
                self.tiltShooter.setSpeed(speed)
                #print("positive speed")

            #elif(operator.getYButton() and (operator.getRightY() < -0.95)):
            #    self.tiltShooter.setSpeed(-speed)
            #elif(operator.getRightY() < -0.95) and (self.tiltShooter.getDegrees() > self.tiltShooter.getMinDegrees()):
            #    self.tiltShooter.setSpeed(-speed)
            #elif(operator.getRightY() > 0.95) and (self.tiltShooter.getDegrees() < self.tiltShooter.getMaxDegrees()):
            #    self.tiltShooter.setSpeed(speed)
            else:
                self.tiltShooter.setSpeed(0.0)
                #print("no speed")
            #print("manually tilt: ", self.tiltShooter.getSpeed())

        # print("Int tilt shooter: degrees:", self.tiltShooter.getDegrees(), " speed: ", self.tiltShooter.getSpeed())

    def getPOVRange(self, value):
        if (value >=0 and value < 45) or (value > 315 and value <= 360):
            return 0
        elif (value > 135) and (value < 225):
            return 180
        else:
            return -1

    def tiltShooterPeriodic(self):
        if not self.tiltShooter:
            return

        speed = self.tiltShooter.getTargetSpeed()
        buffer = self.tiltShooter.getBufferDegrees()

        if (self.tiltShooter.getDegrees() < (self.tiltShooter.getTargetDegrees() - buffer)):
            self.tiltShooter.setSpeed(speed)
        elif (self.tiltShooter.getDegrees() > (self.tiltShooter.getTargetDegrees() + buffer)):
            self.tiltShooter.setSpeed(-speed)
        else:
            self.tiltShooter.setSpeed(0.0)

    def teleopShooter(self, shooterRPM=None, shooterVelocity=None):

        if not self.shooter:
            return

        shooter_mod = 1
        running = 0

        operator = self.operator.xboxController
        rta = self.operator.right_trigger_axis
        shooter_pid = self.shooter.pidController

        if shooterRPM:
            shooterRPM = -shooterRPM
            if self.phase == "DRIVE_PHASE":
                if operator.getRawAxis(rta) > 0.95:
                    shooter_pid.setReference(shooterRPM, rev.CANSparkMax.ControlType.kVelocity)
                else:
                    shooter_pid.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
            elif self.phase == "AS_ROTATE_PHASE" or self.phase == "AS_TILT_PHASE" or self.phase == "AS_FIRE_PHASE":
                shooter_pid.setReference(shooterRPM, rev.CANSparkMax.ControlType.kVelocity)

        elif shooterVelocity:
            shooterVelocity = -shooterVelocity

            if operator.getRawAxis(rta) > 0.95:
                shooter_mod = shooterVelocity
                running = 1
                if (shooter_mod > 1):
                    shooter_mod = 1
            else:
                running = 0
                shooter_mod = 1

            self.shooter.setShooterVelocity(running * shooter_mod)

        else:
            if operator.getRawAxis(rta) > 0.95:
                running = 1
            else:
                running = 0

            if operator.getXButton():
                shooter_mod = 0.4
            else:
                shooter_mod = 1

            self.shooter.setShooterVelocity(running * shooter_mod)

        # print("Shooter velocity equals", self.shooter.encoder.getVelocity())

    def teleopFeeder(self):
        if not self.feeder:
            return

        operator = self.operator.xboxController

        if (self.phase == "AS_FIRE_PHASE"):
            if (self.feeder.hasFired()):
                self.feeder.setFeeder(0.0)
            else:
                self.feeder.setFeeder(self.feeder.feederSpeed)
        else:  # Manual
            if operator.getRightBumper():
                self.feeder.setFeeder(self.feeder.feederSpeed)
            else:
                self.feeder.setFeeder(0.0)

    def teleopClimber(self):

        if not self.climber:
            return

        operator = self.operator.xboxController
        deadzone = self.operator.deadzone

        # self.climber.solenoids.get()
        if operator.getAButtonPressed():
            self.climber.solenoids.toggle()

        #self.climber.setWinch(self.deadzoneCorrection(operator.getLeftY(), deadzone))
        self.climber.setWinch(operator.getLeftY(), operator.getRightY())

    def disabledInit(self):
        
        
        self.autonTimeBase = self.autonTilting1Time
        
        self.autonPhase = "AUTON_1_TILTING"
        
        #print('resetting timers in disabledInit')

        if hasattr(self, 'autonTimer') and self.autonTimer is not None:
            self.autonTimer.reset()

    def autonomousInit(self):

        if not self.auton:
            return

        self.autonTimer = wpilib.Timer()
        self.autonTimer.start()

        self.autonPhase = "AUTON_1_TILTING"
        self.autonTimeBase = self.autonTilting1Time
        if (self.tiltShooter):
            self.tiltShooter.resetPosition()
            self.tiltShooter.setTargetDegrees(self.autonTilt1TargetDegrees)

        #self.theta = None
        #self.rotationSpeed = 1000

        if (self.aimer):
            self.aimer.reset()

        #if (self.intake):
        #    self.intake.extend()

    def autonomousPeriodic(self):

        if not self.auton:
            return

        timer = self.autonTimer.get()

        deadzone = self.driver.deadzone
        speedratio = 1.0

        print("Auton Phase: ", self.autonPhase)
        print("Auton Timer: ", timer)
        
        # Phase transitions
        if timer > self.autonTimeBase and self.autonPhase == "AUTON_1_TILTING":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_1_SPINUP"
            self.autonTimeBase += self.autonTilting1Time

        if timer > self.autonTimeBase and self.autonPhase == "AUTON_1_SPINUP":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_1_FIRING"
            self.autonTimeBase += self.autonSpinUp1Time
        
        if timer > self.autonTimeBase and self.autonPhase == "AUTON_1_FIRING":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_1_ROTATE"
            self.autonTimeBase += self.autonFiring1Time

        if timer > self.autonTimeBase and self.autonPhase == "AUTON_1_ROTATE":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_1_DRIVE"
            self.autonTimeBase += self.autonRotate1Time
        
        if timer > self.autonTimeBase and self.autonPhase == "AUTON_1_DRIVE":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_INTAKE"
            self.autonTimeBase += self.autonDrive1Time

        if timer > self.autonTimeBase and self.autonPhase == "AUTON_INTAKE":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_2_ROTATE"
            self.autonTimeBase += self.autonIntakeTime

        if timer > self.autonTimeBase and self.autonPhase == "AUTON_2_ROTATE":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_2_DRIVE"
            self.autonTimeBase += self.autonRotate2Time
        
        if timer > self.autonTimeBase and self.autonPhase == "AUTON_2_DRIVE":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_2_TILTING"
            if (self.tiltShooter):
                self.tiltShooter.setTargetDegrees(self.autonTilt2TargetDegrees)
            self.autonTimeBase += self.autonDrive2Time

        if timer > self.autonTimeBase and self.autonPhase == "AUTON_2_TILTING":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_2_SPINUP"
            self.autonTimeBase += self.autonTilting2Time

        if timer > self.autonTimeBase and self.autonPhase == "AUTON_2_SPINUP":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_2_FIRING"
            self.autonTimeBase += self.autonSpinUp2Time

        if timer > self.autonTimeBase and self.autonPhase == "AUTON_2_FIRING":
            print("Auton Phase: ", self.autonPhase)
            self.autonPhase = "AUTON_DONE"
            self.autonTimeBase += self.autonFiring2Time

        # Auton Logic
        # Spin up the shooter motor

        # Tilt the hood
        if self.autonPhase == "AUTON_1_TILTING":
            self.tiltShooterPeriodic()
            # Shooter not moving
            # Feeder not moving
            # No intake movement
            self.drivetrain.motors.arcadeDrive(0, 0) # Don't drive

        # Start spinning motor
        if self.autonPhase == "AUTON_1_SPINUP":
            # self.shooter.pidController.setReference(self.shooter.shooterRPM, rev.CANSparkMax.ControlType.kVelocity)
            self.tiltShooterPeriodic()
            self.shooter.setShooterVelocity(-self.autonShootSpeed) #Start spinning shooter
            # Feeder not moving
            # No intake movement
            self.drivetrain.motors.arcadeDrive(0, 0) # Don't drive

        # Activate the feeder/trigger motor
        elif self.autonPhase == "AUTON_1_FIRING":
            self.tiltShooterPeriodic()
            # Keep spinning shooter
            # No intake movement
            self.feeder.setFeeder(self.feeder.feederSpeed) # Trigger shot
            self.drivetrain.motors.arcadeDrive(0, 0) # Don't drive
            self.aimer.reset() # Reset gyro to current heading

        # Turn off the feeder/trigger motors, and rotate to ball
        elif self.autonPhase == "AUTON_1_ROTATE":
            self.tiltShooter.setTargetDegrees(self.autonTilt2TargetDegrees)
            self.tiltShooterPeriodic()
            # Keep spinning shooter
            self.intake.motorOn()
            self.feeder.setFeeder(0.0)

            if (self.autonRotate1TargetDegrees >= -180 and self.autonRotate1TargetDegrees <= 180):
                self.aimer.setError(self.autonRotate1TargetDegrees - self.aimer.getYaw())
                result = (self.aimer.PIDcalc(self.aimer.getError()), 0)
            else:
                result = (0, 0)
            
            rotateSpeed = result[0]
            driveSpeed = result[1]
            
            #rotateSpeed = speedratio * self.deadzoneCorrection(rotateSpeed + self.rotationCorrection, deadzone)
            self.drivetrain.motors.arcadeDrive(rotateSpeed, driveSpeed)

            #self.drivetrain.resetPosition()

        # Drive to ball
        elif self.autonPhase == "AUTON_1_DRIVE":
            self.tiltShooterPeriodic()
            # Keep spinning shooter
            # Feeder not moving
            # Keep spinning intake motor
            self.intake.extend()
            print("Drive Distance (L/R)", self.drivetrain.getLeftInches(), self.drivetrain.getRightInches())
            if(self.drivetrain.getLeftInches() and self.drivetrain.getRightInches()):
                if(self.drivetrain.getLeftInches() < self.autonDrive1Distance or self.drivetrain.getRightInches() < self.autonDrive1Distance):
                    self.drivetrain.motors.arcadeDrive(0, self.autonDriveSpeed) # Drive forward
                else:
                    self.drivetrain.motors.arcadeDrive(0, 0) # Stop driving
            elif(self.drivetrain.getLeftInches() and not self.drivetrain.getRightInches()):
                if(self.drivetrain.getLeftInches() < self.autonDrive1Distance):
                    self.drivetrain.motors.arcadeDrive(0, self.autonDriveSpeed) # Drive forward
                else:
                    self.drivetrain.motors.arcadeDrive(0, 0) # Stop driving
            elif(not self.drivetrain.getLeftInches() and self.drivetrain.getRightInches()):
                if(self.drivetrain.getRightInches() < self.autonDrive1Distance):
                    self.drivetrain.motors.arcadeDrive(0, self.autonDriveSpeed) # Drive forward
                else:
                    self.drivetrain.motors.arcadeDrive(0, 0) # Stop driving
            else:
                self.drivetrain.motors.arcadeDrive(0, self.autonDriveSpeed) # Drive forward

        # Scoop up ball
        elif self.autonPhase == "AUTON_INTAKE":
            # Keep spinning intake motor
            self.intake.retract() # Retract intake
            self.aimer.reset() # Reset gyro to current heading

        # Rotate to target
        elif self.autonPhase == "AUTON_2_ROTATE":
            self.tiltShooterPeriodic()
            # Keep spinning shooter
            # Feeder not moving
            self.intake.motorOff()
            
            if (self.autonRotate2TargetDegrees >= -180 and self.autonRotate2TargetDegrees <= 180):
                self.aimer.setError(self.autonRotate2TargetDegrees - self.aimer.getYaw())
                result = (self.aimer.PIDcalc(self.aimer.getError()), 0)
            else:
                result = (0, 0)
            
            rotateSpeed = result[0]
            driveSpeed = result[1]
            
            #rotateSpeed = speedratio * self.deadzoneCorrection(rotateSpeed + self.rotationCorrection, deadzone)
            self.drivetrain.motors.arcadeDrive(rotateSpeed, driveSpeed)

            #self.drivetrain.resetPosition()

        #Drive to target
        elif self.autonPhase == "AUTON_2_DRIVE":
            self.tiltShooterPeriodic()
            # Keep spinning shooter
            # Feeder not moving
            if(self.drivetrain.getLeftInches() and self.drivetrain.getRightInches()):
                if(self.drivetrain.getLeftInches() < self.autonDrive2Distance or self.drivetrain.getRightInches() < self.autonDrive2Distance):
                    self.drivetrain.motors.arcadeDrive(0, self.autonDriveSpeed) # Drive forward
                else:
                    self.drivetrain.motors.arcadeDrive(0, 0) # Stop driving
            elif(self.drivetrain.getLeftInches() and not self.drivetrain.getRightInches()):
                if(self.drivetrain.getLeftInches() < self.autonDrive2Distance):
                    self.drivetrain.motors.arcadeDrive(0, self.autonDriveSpeed) # Drive forward
                else:
                    self.drivetrain.motors.arcadeDrive(0, 0) # Stop driving
            elif(not self.drivetrain.getLeftInches() and self.drivetrain.getRightInches()):
                if(self.drivetrain.getRightInches() < self.autonDrive2Distance):
                    self.drivetrain.motors.arcadeDrive(0, self.autonDriveSpeed) # Drive forward
                else:
                    self.drivetrain.motors.arcadeDrive(0, 0) # Stop driving
            else:
                self.drivetrain.motors.arcadeDrive(0, self.autonDriveSpeed) # Drive forward
        
        # Re-tilt the hood
        elif self.autonPhase == "AUTON_2_TILTING":
            self.tiltShooterPeriodic()
            # Keep spinning shooter
            # Feeder not moving
            self.drivetrain.motors.arcadeDrive(0, 0) # Don't drive

        # Keep spinning motor
        elif self.autonPhase == "AUTON_2_SPINUP": # Phase may be unnecessary
            self.tiltShooterPeriodic()
            # Keep spinning shooter
            # Feeder not moving
            self.drivetrain.motors.arcadeDrive(0, 0) #  Don't drive

        # Activate the feeder/trigger motor
        elif self.autonPhase == "AUTON_2_FIRING":
            self.tiltShooterPeriodic()
            #keep spinning shooter
            self.feeder.setFeeder(self.feeder.feederSpeed) # Trigger shot
            self.drivetrain.motors.arcadeDrive(0, 0) #  Don't drive

        else:
            # Ignore the tilter
            self.shooter.setShooterVelocity(0) #Stop spinning shooter
            self.drivetrain.motors.arcadeDrive(0, 0) # Don't drive
            self.feeder.setFeeder(0.0)

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

    def autoAimLookup(self, distanceInches):
        if not distanceInches:
            return (0, 0)

        if distanceInches > 480 or distanceInches < 0:
            print("Auto Aim Lookup: distanceInches out of bounds")
            return (0, 0)

        distanceFeet = round(distanceInches / 12, 0)

        velocity = autoAimTable[distanceFeet][0]
        angle = autoAimTable[distanceFeet][1]

        if velocity > self.shooter.getShooterMaxRPM() or velocity < self.shooter.getShooterMinRPM():
            print("Auto Aim Lookup: velocity out of bounds")
            return (0, 0)
        
        if angle > self.tiltShooter.getMaxDegrees() or angle < self.tiltShooter.getMinDegrees():
            print("Auto Aim Lookup: angle out of bounds")
            return (0, 0)

        print("Auto Aim Lookup: velocity: ", velocity, " angle: ", angle)
        return (velocity, angle)


if __name__ == "__main__":
    if sys.argv[1] == 'sim':
        TEST_MODE = True
    wpilib.run(MyRobot)