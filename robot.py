import math
import time
import wpilib
import wpilib.drive
import wpimath.controller
from shooter import Shooter
from wpilib import interfaces
import rev
import robotmap
from navx import AHRS
from aimer import Aimer

#Drive Types
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

        #Create both xbox controlers
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
        self.aimer.reset()
        
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
        
        #Drivetrain
        self.drivetrain = wpilib.drive.DifferentialDrive(self.left_side, self.right_side)
        self.drive = ARCADE

        # Change these depending on the controller
        self.left_trigger_axis = 2 
        self.right_trigger_axis = 3
        #print("running!")


    def robotPeriodic(self):
        pass

    def teleopInit(self):
        self.shooter_mod = 1
        self.running = 0
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
        else:
            self.running = 0 

        if self.operator.getXButton():
            self.shooter_mod = 0.4
        else:
            self.shooter_mod = 1

        #print(self.running * self.shooter_mod)
        self.shooter.set(self.running * self.shooter_mod)
        #print(self.shooter.get())

        #TANK DRIVE
        if (self.drive == TANK):

            #Get left and right joystick values.
            leftspeed = self.driver.getLeftY()
            rightspeed = -(self.driver.getRightY())

            #Invoke deadzone on speed.
            leftspeed = 0.80 * self.deadzone(leftspeed, robotmap.deadzone)
            rightspeed = 0.80 * self.deadzone(rightspeed, robotmap.deadzone)
            
            #Invoke Tank Drive
            self.drivetrain.tankDrive(leftspeed, rightspeed)

        #ARCADE DRIVE
        elif (self.drive == ARCADE):
            if (self.driver.getLeftBumper()):
                self.aimer.reset()

            theta = self.calculateTheta(self.driver.getLeftX(), self.driver.getLeftY())
        
            #print("X = ", -(self.driver.getRightX()), " Y = ", self.driver.getRightY())
            if (self.driver.getRightBumper()):
                self.rotateToTheta(theta)
            else:
                self.drivetrain.arcadeDrive(-self.driver.getRightX(), self.driver.getRightY())

        else: #self.drive == SWERVE
            #Panic
            return

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def rotateToTheta(self, theta):
        angle = self.aimer.getYaw()
        diff = abs(angle - theta)
        correctionFactor = (diff / 10.0)
        if (correctionFactor > 1.0):
            correctionFactor = 1.0
        if (diff > 1):
            if (theta > 0):
                #print("turning left")
                self.drivetrain.arcadeDrive(-(0.5 * correctionFactor), 0)
            else:
                #print("turning right")
                self.drivetrain.arcadeDrive((0.5 * correctionFactor), 0)
    
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
            #print("unknown coordinates (", x, ", ", y, ")")
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
