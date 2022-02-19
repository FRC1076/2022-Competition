import math
import time

import wpilib
import wpilib.drive
import wpimath.controller
from wpilib import interfaces
import rev
from navx import AHRS


from robotconfig import robotconfig
from climber import Climber, SolenoidGroup
#from vision import Vision
from aimer import Aimer
from shooter import Shooter
from controller import Controller

# ROBOT COMPONENTS
BOT_HAS_CLIMBER = False
BOT_HAS_DRIVETRAIN = True
BOT_HAS_GYRO = True
BOT_HAS_SHOOTER = False
DRIVER_HAS_CONTROLLER = True
OPERATOR_HAS_CONTROLLER = True
BOT_HAS_VISION = False

# CONSTANTS
PISTON_REVERSE = wpilib.DoubleSolenoid.Value.kReverse
PISTON_FORWARD = wpilib.DoubleSolenoid.Value.kForward

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):

        self.drivetrain = None
        self.driver = None
        self.operator = None
        self.drivetrain = None
        self.shooter = None
        self.climber = None
        self.aimer = None
        self.vision = None
        self.config = robotconfig

        for key, config in self.config.items():
            if key == 'CONTROLLERS':
                controllers = self.initControllers(config)
                self.driver = controllers[0]
                self.operator = controllers[1]
            if key == 'DRIVETRAIN':
                self.drivetrain = self.initDrivetrain(config)
            if key == 'SHOOTER':
                self.shooter = self.initShooter(config)
            if key == 'CLIMBER':
                self.climber = self.initClimber(config)
            if key == 'AIMER':
                self.aimer = self.initAimer(config)
            if key == 'VISION':
                self.vision = self.initVision(config)


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
        shooter = Shooter(shooter)
        return shooter

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
        aimer = Aimer(ahrs)
        aimer.reset()
        return aimer

    def initVision():
        pass

    def robotPeriodic(self):
        pass

    def teleopInit(self):
        
        if(self.climber): # false if no climber initialized
            # Climber presets
            self.climbRunning = False
            self.t = time.time()

            self.tm = wpilib.Timer()
            self.tm.start()

            self.climber.solenoids.set(PISTON_REVERSE)
        

    def teleopPeriodic(self):
        self.teleopDrivetrain()
        self.teleopIntake()
        self.teleopShooter()
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
        elif (self.drive_type == ARCADE):

            if (driver.getLeftBumper()): # for testing auto-rotate
                self.aimer.reset()

            if (driver.getRightBumper()): # for testing auto-rotate    
                theta = self.aimer.calculateTheta(driver.getLeftX(), driver.getLeftY())            
                result = self.aimer.calcRotationCoordinates(theta)
            else:
                result = (-driver.getRightX(), driver.getRightY())
                
            self.drivetrain.arcadeDrive(result[0], result[1])
            
        else: # self.drive == SWERVE
            # Panic
            return


    def teleopIntake(self):
        pass


    def teleopShooter(self):
        """
        NOTE: This description seems inaccurate!

        Makes the shooter motor spin. Right trigger -> 1, left trigger -> -0.2, 
        x reduces the speed, 
        y reduces the speed more, 
        b reduces the speed even more, 
        a reduces the speed the most
        """

        if not self.shooter:
            return

        operator = self.operator.controller
        rta = self.operator.right_trigger_axis

        if operator.getRawAxis(rta) > 0.95:
            self.running = 1 # does this need to be 'self'?
        else:
            self.running = 0 

        if operator.getXButton():
            shooter_mod = 0.4
        else:
            shooter_mod = 1

        #print(self.running * self.shooter_mod)
        self.shooter.set(self.running * shooter_mod)

    def teleopClimber(self):

        if not self.climber:
            return
        
        operator = self.operator.controller
        deadzone = self.operator.deadzone

        # AUTO CLIMBER
        # NOTE: None of this seems quite right... --mwn
        if self.operator.getAButtonPressed() and self.operator.getBButtonPressed() and self.driver.getAButtonPressed() and self.driver.getBButtonPressed():
            self.climbRunning = True
            self.duration = self.climber.climbActions[self.climber.climbstep][1] 
            self.t = time.time()
    
        if self.climbRunning:
            if self.operator.getAButtonPressed():
                self.climbRunning = False # cancel AutoClimb
            elif time.time() - self.t > self.duration:
                self.climbRunning = False
                self.climber.nextStep()
            else:
                self.climber.stepAction()
        
        # self.climber.solenoids.get()
        if operator.getXButtonPressed():
            self.climber.solenoids.toggle()
        if operator.getBButtonPressed():
            self.right_piston.toggle() # what does this do?

        self.climber.setWinch(self.deadzoneCorrection(operator.getLeftY(), 
                              deadzone))
        
    def autonomousInit(self):
        self.autonTimer = wpilib.Timer()
        self.shooterTimer = wpilib.Timer()
        self.autonTimer.start()
        if(self.aimer):
            self.aimer.reset()
        
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

if __name__ == "__main__":
    wpilib.run(MyRobot)
