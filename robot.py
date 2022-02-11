import math
import time

import wpilib
import wpilib.drive
from wpilib import interfaces
import wpimath.controller

import rev
from navx import AHRS

from shooter import Shooter
import robotmap
import climber
from aimer import Aimer

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

class MyRobot(wpilib.TimedRobot):
    
    # Gyros
    kP = 0.03
    kI = 0.00
    kD = 0.00
    kToleranceDegrees = 2.0


    def robotInit(self):

        # Create both xbox controlers
        self.driver = wpilib.XboxController(0)
        self.operator = wpilib.XboxController(1)

        # Motors
        self.left_motor_1 = rev.CANSparkMax(robotmap.LEFT_LEADER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.left_motor_2 = rev.CANSparkMax(robotmap.LEFT_MIDDLE_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.left_motor_3  = rev.CANSparkMax(robotmap.LEFT_FOLLOWER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.right_motor_1 = rev.CANSparkMax(robotmap.RIGHT_LEADER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.right_motor_2 = rev.CANSparkMax(robotmap.RIGHT_MIDDLE_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.right_motor_3 = rev.CANSparkMax(robotmap.RIGHT_FOLLOWER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        
        #gyros
        self.ahrs = AHRS.create_spi()
        # self.navx = navx.AHRS.create_i2c()
        self.aimer = Aimer(self.ahrs)
        
        shooter = rev.CANSparkMax(robotmap.SHOOTER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.shooter = Shooter(shooter)
        
        self.left_motor_1.setClosedLoopRampRate(1.0)
        self.left_motor_2.setClosedLoopRampRate(1.0)
        self.left_motor_3.setClosedLoopRampRate(1.0)

        self.right_motor_1.setClosedLoopRampRate(1.0)
        self.right_motor_2.setClosedLoopRampRate(1.0)
        self.right_motor_3.setClosedLoopRampRate(1.0)
        #self.shooter.setClosedLoopRampRate(1.0)
        
        self.left_side = wpilib.SpeedControllerGroup(self.left_motor_1, self.left_motor_2, self.left_motor_3)
        self.right_side = wpilib.SpeedControllerGroup(self.right_motor_1, self.right_motor_2, self.right_motor_3)
        
        # Drivetrain
        self.drivetrain = wpilib.drive.DifferentialDrive(self.left_side, self.right_side)


        # Climber
        self.right_winch = rev.CANSparkMax(robotmap.WINCH_RIGHT_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless) # assuming this is a Neo; otherwise it may not be brushless
        self.left_winch = rev.CANSparkMax(robotmap.WINCH_LEFT_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        self.winch = wpilib.SpeedControllerGroup(self.right_winch, self.left_winch)

        self.right_piston = wpilib.DoubleSolenoid(0, wpilib.PneumaticsModuleType.CTREPCM, robotmap.SOLENOID_RIGHT_FORWARD_ID, robotmap.SOLENOID_RIGHT_REVERSE_ID)
        self.left_piston = wpilib.DoubleSolenoid(0, wpilib.PneumaticsModuleType.CTREPCM, robotmap.SOLENOID_LEFT_FORWARD_ID, robotmap.SOLENOID_LEFT_REVERSE_ID)

        self.piston = climber.SolenoidGroup([self.right_piston, self.left_piston])

        self.climber = climber.Climber(self.piston, self.winch)
        self.drive = ARCADE

        # Change these depending on the controller
        self.left_trigger_axis = 2 
        self.right_trigger_axis = 5
        #print("running!")


    def robotPeriodic(self):
        pass

    def teleopInit(self):
        # Shooter presets
        self.shooter_mod = 1
        self.running = 0
        
        # Climber presets
        self.climbRunning = False
        self.t = time.time()

        self.tm = wpilib.Timer()
        self.tm.start()

    def teleopPeriodic(self):
        #print("starting teleop periodic")
        """
        Makes the shooter motor spin. Right trigger -> 1, left trigger -> -0.2, 
        x reduces the speed, y reduces the speed more, b reduces the speed even more, 
        a reduces the speed the most
        """
        if self.operator.getRawAxis(self.right_trigger_axis) > 0.95:
            #print("got trigger")
            self.running = 1
        elif self.operator.getRawAxis(self.left_trigger_axis) > 0.95:
            self.running = -0.2
        else:
            self.running = 0 

        if self.operator.getAButton():
            #print("got A button!")
            self.shooter_mod = 0.2
        elif self.operator.getBButton():
            self.shooter_mod = 0.4
        elif self.operator.getYButton():
            self.shooter_mod = 0.6
        elif self.operator.getXButton():
            self.shooter_mod = 0.8
        else:
            self.shooter_mod = 1

        self.shooter.set(self.running * self.shooter_mod)

        #TANK DRIVE
        if (self.drive == TANK):

            #Get left and right joystick values.

            leftspeed = self.driver.getRightX()

            rightspeed = self.driver.getRightY()
            #leftspeed = 0.5
            #rightspeed = 0.5
            #Invoke deadzone on speed.
            leftspeed = 0.80 * self.deadzone(leftspeed, robotmap.deadzone)
            rightspeed = 0.80 * self.deadzone(rightspeed, robotmap.deadzone)
            #print("Right Speed: ", rightspeed)
            #print("Left Speed: ", leftspeed)
            #Invoke Tank Drive
            self.drivetrain.tankDrive(leftspeed, rightspeed)

        #ARCADE DRIVE
        elif (self.drive == ARCADE):
            self.ahrs.reset()
            theta = self.calculateTheta(self.driver.getLeftX(), self.driver.getLeftY())
            #self.driver.rotateVector(self.driver.getLeftX(), self.driver.getLeftY(), theta)
            print("(", self.driver.getLeftX(), ", ", self.driver.getLeftY(), ") = ", theta)
            
            self.rotateToTheta(theta)

            '''
            #print("0", self.turnController.calculate(self.AHRS.getYaw(),0.0))
            #print("90", self.turnController.calculate(self.AHRS.getYaw(),90.0))
            #print("179.9", self.turnController.calculate(self.AHRS.getYaw(),179.9))
            #print("-90", self.turnController.calculate(self.AHRS.getYaw(),-90.0))
            #print("RightX", self.driver.getRightX())
            if self.tm.hasPeriodPassed(1.0):
                #print("NavX Gyro", self.AHRS.getYaw(), self.AHRS.getAngle())
                rotateToAngle = False
            if self.driver.getRightBumper():
                print("Right bumper")
                self.AHRS.reset()
                rotateToAngle = False
            if self.driver.getYButton():
                print("Y button")
                setPoint = 0.0
                rotateToAngle = True
            elif self.driver.getBButton():
                print("B button")
                setPoint = 90.0
                rotateToAngle = True
            elif self.driver.getAButton():
                print("A button")
                setPoint = 179.9
                rotateToAngle = True
            elif self.driver.getXButton():
                print("X Button")
                setPoint = -90.0
                rotateToAngle = True
            else:
                rotateToAngle = False

            if rotateToAngle:
                currentRotationRate = self.turnController.calculate(self.AHRS.getYaw(), setPoint)
                print(setPoint, " ", currentRotationRate)
            else:
                self.turnController.reset()
                currentRotationRate = self.driver.getRightX()
                #print("No Button ", currentRotationRate)

            #self.drivetrain.arcadeDrive(-self.driver.getRightY(), currentRotationRate)
            self.drivetrain.arcadeDrive(currentRotationRate, -self.driver.getRightY())
            '''
            #self.turnController.reset()
            currentRotationRate = self.driver.getRightX()
            self.drivetrain.arcadeDrive(-self.driver.getRightY(), currentRotationRate)

        else: #self.drive == SWERVE
            #Panik
            return

        if self.operator.getAButtonPressed() and self.operator.getBButtonPressed() and self.driver.getAButtonPressed() and self.driver.getBButtonPressed():
            self.climbRunning = True
            self.duration = self.climber.climbActions[self.climber.climbstep][1]
            self.t = time.time()
        
        if self.climbRunning:
            if self.operator.getAButtonPressed():
                self.climbRunning = False
            elif time.time()-self.t > self.duration:
                self.climbRunning = False
                self.climber.nextStep()
            else:
                self.climber.stepAction()
        
        else:
            self.climber.setWinch(self.operator.getLeftY())
            self.climber.climberOff()
        

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def rotateToTheta(self, theta):
        self.aimer.reset()
        amount = self.aimer.calculate(theta)
        print("theta = ", theta, " amount = ", amount)
        #self.drivetrain.arcadeDrive(0, amount)


        '''
        while(abs(theta - self.AHRS.getYaw()) > 1.0):
            currentRotationRate = self.turnController.calculate(self.AHRS.getYaw(), theta)
            #self.drivetrain.arcadeDrive(currentRotationRate, -self.driver.getRightY())
            #self.drivetrain.arcadeDrive(currentRotationRate, -0.01)
            print("theta = ", theta, "yaw = ", self.AHRS.getYaw())
            #print(currentRotationRate)
        '''

    def calculateTheta(self, x, y):
        y = -y
        theta = 0.0
        absY = abs(float(y))
        absX = abs(float(x))
        if(x == 0) and (y >= 0):
            theta = 0.0
        elif(x == 0) and (y <= 0):
            theta = 179.9
        elif(x > 0) and (y >= 0):
            theta = 90 - (math.atan(absY/absX)*180/math.pi)
        elif(x < 0) and (y >= 0):
            theta = -(90 - (math.atan(absY/absX)*180/math.pi))
        elif(x > 0) and (y < 0):
            theta = 90 + (90 - (math.atan(absY/absX)*180/math.pi))
        elif(x < 0) and (y < 0):
            theta = -(90 + (math.atan(absY/absX)*180/math.pi))
        else:
            theta = 0.0
            print("unknown coordinates (", x, ", ", y, ")")
        return(theta)

    def deadzone(self, val, deadzone): 
        """
        Given the deadzone value x, the deadzone both eliminates all
        values between -x and x, and scales the remaining values from
        -1 to 1, to (-1 + x) to (1 - x)
        """
        if abs(val) < deadzone:
            return 0
        elif val < (0):
            x = ((abs(val) - deadzone)/(1-deadzone))
            return (-x)
        else:
            x = ((val - deadzone)/(1-deadzone))
            return (x)

# You do need to include these lines for the code to run
if __name__=="__main__":
    wpilib.run(MyRobot)
