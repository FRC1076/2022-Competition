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

from robotconfig import robotconfig
import climber  # not needed?
from climber import Climber, WinchGroup
from vision import Vision
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
        self.autonTiltingTime = config['TILTING_TIME']
        self.autonSpinUpTime = config['SPINUP_TIME']
        self.autonFiringTime = config['FIRING_TIME']
        self.autonBackupTime = config['BACKUP_TIME']
        self.autonTiltTargetDegrees = config['TILT_TARGET_DEGREES']
        self.autonShootSpeed = config['SHOOT_SPEED']
        return True

    def initDrivetrain(self, config):
        left_motors = []
        right_motors = []
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushed

        for controller_id in config['LEFT'].values():
            left_motors.append(rev.CANSparkMax(controller_id, motor_type))

        for controller_id in config['RIGHT'].values():
            right_motors.append(rev.CANSparkMax(controller_id, motor_type))

        self.drive_type = config['DRIVETYPE']  # side effect!

        self.rotationCorrection = config['ROTATION_CORRECTION']

        # Create Controller Groups
        left_side = wpilib.MotorControllerGroup(*left_motors)
        right_side = wpilib.MotorControllerGroup(*right_motors)

        # Create Drivetrain
        return wpilib.drive.DifferentialDrive(left_side, right_side)

    def initShooter(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        shooter_motor = rev.CANSparkMax(config['SHOOTER_ID'], motor_type)
        shooter = Shooter(shooter_motor, config['SHOOTER_RPM'])
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
        #motor = rev.CANSparkMax(config['INTAKE_MOTOR_ID'], motor_type)
        solenoid = wpilib.DoubleSolenoid(0,
                                         pneumatics_module_type,
                                         config['INTAKE_SOLENOID_FORWARD_ID'],
                                         config['INTAKE_SOLENOID_REVERSE_ID'])

        #return Intake(solenoid, motor)
        return Intake(solenoid)

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

    def robotPeriodic(self):
        self.dashboard.putNumber('Periods', self.periods)
        self.periods += 1
        self.dashboard.putString('Phase ', '' + self.phase)
        if (self.vision):
            self.dashboard.putBoolean('Vision Has Target', self.vision.hasTargets())
            if (self.vision.hasTargets()):
                self.dashboard.putNumber('Vision Pitch', self.vision.getSmoothYaw())
                self.dashboard.putNumber('Vision Yaw', self.vision.getSmoothPitch())
                self.dashboard.putNumber('Vision Distance', self.vision.getDistanceFeet())
                self.dashboard.putNumber('Vision # Targets', self.vision.getNumTargets())
        if (self.aimer):
            # self.dashboard.putNumber('Aimer In Range (T/F)', self.aimer.getInRange())
            self.dashboard.putNumber('Aimer Yaw (Degrees)', self.aimer.getYaw())
            if (self.aimer.getTheta() is not None):
                self.dashboard.putNumber('Aimer Target (Degrees)', self.aimer.getTheta())
        if (self.tiltShooter):
            self.dashboard.putNumber('Tilt (Degrees)', self.tiltShooter.getDegrees())
            self.dashboard.putNumber('Tilt Target (Degrees)', self.tiltShooter.getTargetDegrees())
        if (self.shooter):
            self.dashboard.putNumber('Shooter Speed (RPM)', self.shooter.encoder.getVelocity())
        if (self.feeder):
            self.dashboard.putBoolean('Feeder Has Fired', self.feeder.hasFired())

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
            self.teleopVision()
            self.teleopDrivetrain()
            self.teleopIntake()
            self.teleopTiltShooter()
            # self.teleopShooter()
            self.teleopShooter(shooterRPM=self.shooter.shooterRPM)
            self.teleopFeeder()
            self.teleopClimber()
        elif (self.phase == "AS_ROTATE_PHASE"):
            if (driver.getRawAxis(rta) > 0.95):
                self.phase = "DRIVE_PHASE"
                self.teleopDrivetrain()
            else:
                if (self.aimer.getInRange(self.aimer.getTheta())):
                    self.phase = "AS_TILT_PHASE"
                    self.teleopTiltShooter()
                else:
                    self.teleopDrivetrain()
        elif (self.phase == "AS_TILT_PHASE"):
            if (driver.getRawAxis(rta) > 0.95):
                self.phase = "DRIVE_PHASE"
                self.teleopDrivetrain()
            else:
                if (self.tiltShooter.isNearTarget()):
                    self.phase = "AS_FIRE_PHASE"
                else:
                    self.teleopTiltShooter()
                self.teleopShooter(shooterRPM=self.shooter.shooterRPM)
        elif (self.phase == "AS_FIRE_PHASE"):
            if (driver.getRawAxis(rta) > 0.95):
                self.phase = "DRIVE_PHASE"
                self.teleopDrivetrain()
            else:
                self.teleopShooter(shooterRPM=self.shooter.shooterRPM)
                self.teleopFeeder()
                if (self.feeder.hasFired()):
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

            if (self.vision and self.aimer and self.tiltShooter):
                if (self.phase == "DRIVE_PHASE"):
                    if driver.getLeftBumper():  # for testing auto-rotate
                        self.aimer.reset()
                    if (driver.getRightBumper()):
                        self.aimer.setTheta(self.vision.getSmoothYaw())
                        self.tiltShooter.setTargetDegrees(self.vision.calculateAngle(10))
                        if (self.vision.hasTargets()):
                            if (self.aimer.getTheta() is not None):
                                result = self.aimer.calculateDriveSpeeds(self.aimer.getTheta())
                                self.phase = "AS_ROTATE_PHASE"
                elif (self.phase == "AS_ROTATE_PHASE"):
                    if (self.aimer.getTheta() is not None):
                        result = self.aimer.calculateDriveSpeeds(self.aimer.getTheta())
                    else:
                        print("should never happen")
                        self.phase = "DRIVE_PHASE"
                else:
                    print("In AS_AIMER_PHASE: should never happen")
                    self.phase = "DRIVE_PHASE"

                    # print(result)
            rotateSpeed = result[0]
            driveSpeed = result[1]

            #print(rotateSpeed, driveSpeed)
            rotateSpeed = speedratio * self.deadzoneCorrection(rotateSpeed + self.rotationCorrection, deadzone)
            driveSpeed = speedratio * self.deadzoneCorrection(driveSpeed, deadzone)

            #print(rotateSpeed, driveSpeed)
            self.drivetrain.arcadeDrive(rotateSpeed, driveSpeed)

        else:  # self.drive == SWERVE
            # Panic
            return

    def teleopIntake(self):
        '''
        Manually controls the intake solenoid and motor.
        The left bumper toggles the position of the solenoid,
        and the left trigger turns the motor on when pressed
        in fully.
        '''
        if self.intake is None:
            return

        print("In teleopIntake")
        operator = self.operator.xboxController
        lta = self.operator.left_trigger_axis

        if operator.getLeftBumper():
            self.intake.toggle()
            print("toggling intake")
        '''
        if operator.getRawAxis(lta) > 0.95:
            self.intake.motorOn()
        else:
            self.intake.motorOff()
        '''

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
            if (operator.getXButton()):
                self.tiltShooter.resetPosition()
            elif (operator.getYButton()) and (self.getPOVRange(operator.getPOV()) == 180):
                 self.tiltShooter.setSpeed(-speed)
            elif (self.getPOVRange(operator.getPOV()) == 180) and (self.tiltShooter.getDegrees() > self.tiltShooter.getMinDegrees()):
                self.tiltShooter.setSpeed(-speed)
            elif (self.getPOVRange(operator.getPOV()) == 0) and (self.tiltShooter.getDegrees() < self.tiltShooter.getMaxDegrees()):
                self.tiltShooter.setSpeed(speed)

            #elif(operator.getYButton() and (operator.getRightY() < -0.95)):
            #    self.tiltShooter.setSpeed(-speed)
            #elif(operator.getRightY() < -0.95) and (self.tiltShooter.getDegrees() > self.tiltShooter.getMinDegrees()):
            #    self.tiltShooter.setSpeed(-speed)
            #elif(operator.getRightY() > 0.95) and (self.tiltShooter.getDegrees() < self.tiltShooter.getMaxDegrees()):
            #    self.tiltShooter.setSpeed(speed)
            else:
                self.tiltShooter.setSpeed(0.0)
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
            elif self.phase == "AS_TILT_PHASE" or self.phase == "AS_FIRE_PHASE":
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

            self.shooter.set(running * shooter_mod)

        else:
            if operator.getRawAxis(rta) > 0.95:
                running = 1
            else:
                running = 0

            if operator.getXButton():
                shooter_mod = 0.4
            else:
                shooter_mod = 1

            self.shooter.set(running * shooter_mod)

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
        
        print('resetting timers in disabledInit')

        if hasattr(self, 'autonTimer') and self.autonTimer is not None:
            self.autonTimer.reset()
        
        if hasattr(self, 'shooterTimer') and self.shooterTimer is not None:
            self.shooterTimer.reset()

    def autonomousInit(self):

        if not self.auton:
            return

        self.autonPhase = "AUTON_TILTING"
        self.theta = None
        self.rotationSpeed = 1000

        if (self.tiltShooter):
            self.tiltShooter.resetPosition()
            self.tiltShooter.setTargetDegrees(self.autonTiltTargetDegrees)

        self.autonTimer = wpilib.Timer()
        self.shooterTimer = wpilib.Timer()

        self.autonTimer.start()

        if (self.aimer):
            self.aimer.reset()

        if (self.intake):
            self.intake.extend()

    def autonomousPeriodic(self):

        if not self.auton:
            return

        # self.autonForwardAndBack()
        #self.comp1Auton()

        print("In autonomousPeriodic")
        self.comp1AutonSimple()

    def autonForwardAndBack(self):
        driver = self.driver.xboxController
        if driver.getLeftBumper() and driver.getRightBumper():
            if self.autonTimer.get() < 1.0:
                self.drivetrain.arcadeDrive(0, -0.75)
            elif 1.0 <= self.autonTimer.get() < 2.0:
                self.drivetrain.arcadeDrive(0, 0.75)
            elif 2.0 <= self.autonTimer.get() < 3.0:
                theta = self.aimer.calculateTheta(driver.getLeftX(), driver.getLeftY())
                result = self.aimer.calcRotationCoordinates(theta)
                self.drivetrain.arcadeDrive(result[0], result[1])

    def comp1AutonSimple(self):

        timer = self.autonTimer.get()

        print("Auton Phase: ", self.autonPhase)
        print("Auton Timer: ", timer)

        # Phase transitions
        if timer > self.autonTiltingTime and self.autonPhase == "AUTON_TILTING":
            self.autonPhase = "AUTON_SPINUP"

        if timer > self.autonSpinUpTime + self.autonTiltingTime and self.autonPhase == "AUTON_SPINUP":
            self.autonPhase = "AUTON_FIRING"
        
        if timer > self.autonFiringTime + self.autonSpinUpTime + self.autonTiltingTime and self.autonPhase == "AUTON_FIRING":
            self.autonPhase = "AUTON_DRIVE"
        
        if timer > self.autonBackupTime + self.autonFiringTime + self.autonSpinUpTime + self.autonTiltingTime and self.autonPhase == "AUTON_DRIVE":
            self.autonPhase = "AUTON_DONE"

        # Auton Logic
        # Spin up the shooter motor

        if self.autonPhase == "AUTON_TILTING":
            self.tiltShooterPeriodic()

        if self.autonPhase == "AUTON_SPINUP":
            # self.shooter.pidController.setReference(self.shooter.shooterRPM, rev.CANSparkMax.ControlType.kVelocity)
            self.shooter.set(-self.autonShootSpeed)

        # Activate the feeder/trigger motor
        elif self.autonPhase == "AUTON_FIRING":
            # self.shooter.pidController.setReference(self.shooter.shooterRPM, rev.CANSparkMax.ControlType.kVelocity)
            self.feeder.setFeeder(self.feeder.feederSpeed)
            self.shooter.set(-self.autonShootSpeed)

        # Turn off the shooter and feeder/trigger motors, and drive backwards
        elif self.autonPhase == "AUTON_DRIVE":
            self.shooter.pidController.setReference(0.0, rev.CANSparkMax.ControlType.kVelocity)
            self.feeder.setFeeder(0.0)
            
            self.drivetrain.arcadeDrive(0, 0.8)
        

    def comp1Auton(self):

        backupTime = 0.5

        print(self.autonPhase)
        # Phase transitions
        if self.autonTimer.get() > backupTime and self.autonPhase == "AUTON_DRIVE":
            self.autonPhase = "AUTON_ROTATE"
            self.theta = self.vision.getSmoothYaw()

        if self.rotationSpeed == 0 and self.autonPhase == "AUTON_ROTATE":
            self.autonPhase = "AUTON_TILT"

        if self.tiltShooter.isNearTarget() and self.autonPhase == AUTON_TILT:
            self.autonPhase = "AUTON_SHOOT"

        # Auton logic
        if self.autonPhase == "AUTON_DRIVE":
            self.drivetrain.arcadeDrive(0, 0.8)

        if self.autonPhase == "AUTON_ROTATE":
            if (self.vision and self.aimer and self.theta):
                (self.rotationSpeed, speed) = self.aimer.calculateDriveSpeeds(self.theta)
                self.drivetrain.arcadeDrive(self.rotationSpeed, speed)

        if self.autonPhase == "AUTON_TILT":
            self.tiltShooterPeriodic()
            self.shooter.pidController.setReference(self.autonShooterRPM, rev.CANSparkMax.ControlType.kVelocity)

        if self.autonPhase == "AUTON_SHOOT":
            if (self.feeder.hasFired()):
                self.feeder.setFeeder(0.0)
                self.shooter.pidController.setReference(0, rev.CANSparkMax.ControlType.kVelocity)
            else:
                self.feeder.setFeeder(0.4)

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
    if sys.argv[1] == 'sim':
        TEST_MODE = True
    wpilib.run(MyRobot)
