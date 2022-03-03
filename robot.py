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
import climber # not needed?
from climber import Climber, SolenoidGroup
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

# Shooting phases
PHASE_0 = 0
PHASE_1 = 1
PHASE_2 = 2
PHASE_3 = 3

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        self.drivetrain = None
        self.driver = None
        self.operator = None
        self.shooter = None
        self.tiltShooter = None
        self.intake = None
        self.climber = None
        self.aimer = None
        self.vision = None
        self.tester = None
        self.theta = None

        if TEST_MODE:
            self.tester = Tester(self)         
            self.config = self.tester.getTestConfig()
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

        if TEST_MODE:
            self.tester.initTestTeleop()
            self.tester.testCodePaths()

    def initControllers(self, config):
        ctrls = {}
        print(config)
        for ctrlConfig in config.values():
            print(ctrlConfig)
            id = ctrlConfig['ID']
            ctrl = wpilib.XboxController(id)
            dz = ctrlConfig['DEADZONE']
            lta = ctrlConfig['LEFT_TRIGGER_AXIS']
            rta = ctrlConfig['RIGHT_TRIGGER_AXIS']
            ctrls[id] = Controller(ctrl, dz, lta, rta)
        return ctrls

    def initDrivetrain(self, config):
        left_motors = []
        right_motors = []
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushed

        for id in config['LEFT'].values():
            left_motors.append(rev.CANSparkMax(id, motor_type))

        for id in config['RIGHT'].values():
            right_motors.append(rev.CANSparkMax(id, motor_type))

        self.drive_type = config['DRIVETYPE'] # side effect!

        # Create Controller Groups
        left_side = wpilib.MotorControllerGroup(*left_motors)
        right_side = wpilib.MotorControllerGroup(*right_motors)

        # Create Drivetrain
        return wpilib.drive.DifferentialDrive(left_side, right_side)

    def initShooter(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        shooter = rev.CANSparkMax(config['SHOOTER_ID'], motor_type)
        return Shooter(shooter)
        
    def initFeeder(self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        feeder = rev.CANSparkMax(config['FEEDER_ID'], motor_type)
        return Feeder(feeder)

    def initTiltShooter(self, config):
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        tiltShooter = rev.CANSparkMax(config['TILTSHOOTER_ID'], motor_type)
        tiltShooter = TiltShooter(tiltShooter, config['ROTATIONS_PER_360'], config['MIN_DEGREES'], config['MAX_DEGREES'], config['BUFFER_DEGREES'], config['SPEED'])
        return tiltShooter

    def initIntake (self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        pneumatics_module_type = wpilib.PneumaticsModuleType.CTREPCM
        motor = rev.CANSparkMax(config['INTAKE_MOTOR_ID'], motor_type)
        solenoid = wpilib.DoubleSolenoid(0, 
            pneumatics_module_type, 
            config['INTAKE_SOLENOID_FORWARD_ID'], 
            config['INTAKE_SOLENOID_REVERSE_ID'])
        
        return Intake(solenoid, motor)


    def initClimber (self, config):
        # assuming this is a Neo; otherwise it may not be brushless
        winch_motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        pneumatics_module_type = wpilib.PneumaticsModuleType.CTREPCM

        right_winch = rev.CANSparkMax(config['WINCH_RIGHT_ID'], winch_motor_type)
        left_winch = rev.CANSparkMax(config['WINCH_LEFT_ID'], winch_motor_type)
        winch = wpilib.MotorControllerGroup(right_winch, left_winch)

        right_piston = wpilib.DoubleSolenoid(0, 
            pneumatics_module_type, 
            config['SOLENOID_RIGHT_FORWARD_ID'], 
            config['SOLENOID_RIGHT_REVERSE_ID'])
        left_piston = wpilib.DoubleSolenoid(0, 
             pneumatics_module_type, 
             config['SOLENOID_LEFT_FORWARD_ID'], 
             config['SOLENOID_LEFT_REVERSE_ID'])

        piston = SolenoidGroup([right_piston, left_piston])
        return Climber(piston, winch)

    def initAimer(self, config):
        ahrs = AHRS.create_spi()
        # navx = navx.AHRS.create_i2c()
        aimer = Aimer(ahrs, config['AIMING_ROTATION_SPEED'], config['AIMING_ACCURACY_DEGREES'])
        #aimer = Aimer(ahrs, 0.6, 3)
        aimer.reset()
        return aimer

    def initVision(self, config):
        self.camera = Vision()
        # NetworkTables.initialize(server="10.10.76.2")
        # self.vision_table = NetworkTables.getTable("photonvision/mmal_service_16.1")

    def robotPeriodic(self):
        pass

    def teleopInit(self):
        
        if self.drivetrain:
            self.phase = PHASE_0

        if self.climber: # false if no climber initialized
            # Climber presets
            self.climbRunning = False
            self.t = time.time()

            self.tm = wpilib.Timer()
            self.tm.start()

            self.climber.solenoids.set(climber.kReverse)

        if self.tiltShooter:
            self.tiltShooter.setTargetDegrees(10)
            self.tiltShooter.setManualTiltShooter(False)

    def teleopPeriodic(self):
        self.teleopVision()
        self.teleopDrivetrain()
        self.teleopIntake()
        self.teleopShooter()
        self.teleopTiltShooter()
        self.teleopClimber()

    def teleopDrivetrain(self):
        if(not self.drivetrain):
            return
    
        driver = self.driver.xboxController
        deadzone = self.driver.deadzone

        #TANK DRIVE
        if (self.drive_type == TANK):
            speedratio = 0.8 # ratio of joystick position to motor speed

            #Get left and right joystick values.
            leftspeed = driver.getLeftY()
            rightspeed = -(driver.getRightY())

            #Eliminate deadzone and correct speed
            leftspeed = speedratio * self.deadzoneCorrection(leftspeed, deadzone)
            rightspeed = speedratio * self.deadzoneCorrection(rightspeed, deadzone)
            
            #Invoke Tank Drive
            self.drivetrain.tankDrive(leftspeed, rightspeed)

        #ARCADE DRIVE
        elif self.drive_type == ARCADE:
            speedratio = 0.8  # ratio of joystick position to motor speed

            if driver.getLeftBumper():  # for testing auto-rotate
                if(self.aimer):
                    self.aimer.reset()
            
            if (self.phase == PHASE_0): # Manual mode
                print("In phase 0")
                if (driver.getRightBumper() and (self.vision)): # Automatically aim and shoot
                    # theta = self.aimer.calculateTheta(driver.getLeftX(), driver.getLeftY())
                    self.theta = self.camera.get_smooth_yaw()
                    if ((self.theta != None) and (self.aimer)):
                        self.phase = PHASE_1
                        result = self.aimer.calcRotationCoordinates(self.theta)
                    else:
                        result = (-driver.getRightX(), driver.getRightY())
                else: # Stay in manual mode
                    result = (-driver.getRightX(), driver.getRightY())
            elif (self.phase == PHASE_1): # Rotate to target
                print("In phase 1")
                if (self.theta == None): # Should never happen
                    self.phase = PHASE_0
                    result = (-driver.getRightX(), driver.getRightY())
                    # CONSIDER STARTING FLYWHEEL
                else: # Rotate towards target
                    result = self.aimer.calcRotationCoordinates(self.theta)
                    #print(self.theta, " ", result)
                    if(result[0] == 0.0): # Target aquired FIX WITH A BETTER TEST
                        if (not(self.tiltShooter)): # No tilt-shooter. Skip step.
                            self.phase = PHASE_3
                            result = (0, 0)
                        else: # Set target for shooting
                            self.phase = PHASE_2
                            self.tiltShooter.setTargetDegrees(10) # ASK VISION REAL NUMBER
                            self.tiltShooter.setManualTiltShooter(False)
                            result = (0, 0)
            elif (self.phase == PHASE_2): # Tilt tilt-shooter
                print("In phase 2")
                if (not(self.tiltShooter)): # Should never happen
                    self.phase = PHASE_3
                else: # There IS a tilt-shooter
                    if(self.tiltShooter.getNearTarget()): # Tilter stopped moving
                        self.phase = PHASE_3
                result = (0, 0)
            elif (self.phase == PHASE_3): # Fire missiles
                print("In phase 3")
                print("Fire Missiles")
                self.phase = PHASE_0
                result = (0, 0)
            
            #print(result)
            rotateSpeed = result[0]
            driveSpeed = result[1]

            #print(rotateSpeed, driveSpeed)
            #rotateSpeed = speedratio * self.deadzoneCorrection(rotateSpeed, deadzone)
            #driveSpeed = speedratio * self.deadzoneCorrection(driveSpeed, deadzone)

            self.drivetrain.arcadeDrive(rotateSpeed, driveSpeed)
            
        else: # self.drive == SWERVE
            # Panic
            return


    def teleopIntake(self):
        '''
        Manually controls the intake solenoid and motor.
        The left bumper toggles the position of the solenoid,
        and the left trigger turns the motor on when pressed
        in fully.
        '''
        if self.intake == None:
            return
        
        operator = self.operator.xboxController
        lta = self.operator.left_trigger_axis

        if operator.getLeftBumper():
            self.intake.toggle()
        
        if operator.getRawAxis(lta) > 0.95:
            self.intake.motorOn()
        else:
            self.intake.motorOff()

    def teleopVision(self):
        if(not self.vision):
            return

        result = self.camera.get_latest_result()
        # yaw = self.vision_table.getNumber("targetPitch", )
        #print(self.camera.get_yaw_degrees(), self.camera.get_smooth_yaw())

    def teleopShooter(self, shooterVelocity = -1.0):
        """
        Makes the shooter motor spin. Right trigger activates the trigger
        The X Button reduces the speed.

        The right bumper runs the feeder motor.
        """

        if not self.shooter:
            return

        operator = self.operator.xboxController
        rta = self.operator.right_trigger_axis

        if shooterVelocity != -1.0:

            if operator.getRawAxis(rta) > 0.95:
                shooter_mod = shooterVelocity
                running = 1
                if(shooter_mod > 1):
                    shooter_mod = 1
            else:
                running = 0
                shooter_mod = 1
                
        else:
            if operator.getRawAxis(rta) > 0.95:
                running = 1
            else:
                running = 0 

            if operator.getXButton():
                shooter_mod = 0.4
            else:
                shooter_mod = 1

        # Feeder
        if operator.getRightBumper():
            self.feeder.setFeeder(0.4)
        else:
            self.feeder.setFeeder(0.0)

        self.shooter.set(running * shooter_mod)
    
    def teleopTiltShooter(self):
        if not self.tiltShooter:
            return

        speed = self.tiltShooter.getTargetSpeed()
        buffer = self.tiltShooter.getBufferDegrees()
        lta = self.operator.left_trigger_axis
        operator = self.operator.xboxController
        
        if(self.tiltShooter.manualTiltShooter): # Adjusting tiltShooter mannual
            if(operator.getRightY() > 0.95) and (self.tiltShooter.getDegrees() < self.tiltShooter.getMaxDegrees()):
                self.tiltShooter.setSpeed(speed)
            elif(operator.getRightY() < -0.95) and (self.tiltShooter.getDegrees() > self.tiltShooter.getMinDegrees()):
                if((self.tiltShooter.getSpeed() != 0) and (self.tiltShooter.getVelocity() == 0)): # Hit the limit switch
                    self.tiltShooter.resetPosition() # Reset base position
                    self.tiltShooter.setSpeed(0.0)
                else:
                    self.tiltShooter.setSpeed(-speed)
            else:
                self.tiltShooter.setSpeed(0.0)
        else: # Adjusting tiltShooter automatically with targetDegrees
            if (self.tiltShooter.getDegrees() < (self.tiltShooter.getTargetDegrees() - buffer)):
                self.tiltShooter.setSpeed(speed)
            elif (self.tiltShooter.getDegrees() > (self.tiltShooter.getTargetDegrees() + buffer)):
                if((self.tiltShooter.getSpeed() != 0) and (self.tiltShooter.getVelocity() == 0)): # Hit the limit switch
                    self.tiltShooter.resetPosition() # Reset base position
                    self.tiltShooter.setSpeed(0.0)
                else:
                    self.tiltShooter.setSpeed(-speed)
            else:
                self.tiltShooter.setSpeed(0.0)
                self.tiltShooter.setManualTiltShooter(True)

    def teleopClimber(self):

        if not self.climber:
            return
        
        operator = self.operator.xboxController
        deadzone = self.operator.deadzone

        # AUTO CLIMBER
        # NOTE: None of this seems quite right... --mwn
        if operator.getAButtonPressed() and operator.getBButtonPressed():
            self.climbRunning = True
            self.duration = self.climber.climbActions[self.climber.climbstep][1] 
            self.t = time.time()
    
        if self.climbRunning:
            if operator.getAButtonPressed():
                self.climbRunning = False # cancel AutoClimb
            elif time.time() - self.t > self.duration:
                self.climbRunning = False
                self.climber.nextStep()
            else:
                self.climber.stepAction()
        
        # self.climber.solenoids.get()
        if operator.getXButtonPressed():
            self.climber.solenoids.toggle()

        self.climber.setWinch(self.deadzoneCorrection(operator.getLeftY(), 
                              deadzone))
        
    def autonomousInit(self):
        self.autonTimer = wpilib.Timer()
        self.shooterTimer = wpilib.Timer()
        self.autonTimer.start()
        if(self.aimer):
            self.aimer.reset()

        if(self.intake):
            self.intake.extend()
        
    def autonomousPeriodic(self):
        self.autonForwardAndBack()

    def autonForwardAndBack(self):
        driver = self.driver.xboxController
        if(driver.getLeftBumper() and driver.getRightBumper()):
            if(self.autonTimer.get() < 1.0):
                self.drivetrain.arcadeDrive(0, -0.75)
            elif(self.autonTimer.get() >= 1.0 and self.autonTimer.get() < 2.0):
                self.drivetrain.arcadeDrive(0, 0.75)
            elif(self.autonTimer.get() >= 2.0 and self.autonTimer.get() < 3.0):
                theta = self.aimer.calculateTheta(driver.getLeftX(), driver.getLeftY())
                result = self.aimer.calcRotationCoordinates(theta)
                self.drivetrain.arcadeDrive(result[0], result[1])

    def deadzoneCorrection(self, val, deadzone): 
        """
        Given the deadzone value x, the deadzone both eliminates all
        values between -x and x, and scales the remaining values from
        -1 to 1, to (-1 + x) to (1 - x)
        """
        if abs(val) < deadzone:
            return 0
        elif val < 0:
            x = (abs(val) - deadzone)/(1-deadzone)
            return -x
        else:
            x = (val - deadzone)/(1-deadzone)
            return x

    def logResult(self, *result):
        if (TEST_MODE):
            print(result)


if __name__ == "__main__":
    if sys.argv[1] == 'sim':
        TEST_MODE = True
    wpilib.run(MyRobot)
