import time

import wpilib
import wpilib.drive
import rev

from shooter import Shooter
import robotmap
import climber

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Create both xbox controllers
        self.driver = wpilib.XboxController(0)
        self.operator = wpilib.XboxController(1)

        # Motors
        self.left_motor_1 = rev.CANSparkMax(robotmap.LEFT_LEADER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        # self.left_motor_2 = rev.CANSparkMax(robotmap.LEFT_MIDDLE_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.left_motor_3  = rev.CANSparkMax(robotmap.LEFT_FOLLOWER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.right_motor_1 = rev.CANSparkMax(robotmap.RIGHT_LEADER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        # self.right_motor_2 = rev.CANSparkMax(robotmap.RIGHT_MIDDLE_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        self.right_motor_3 = rev.CANSparkMax(robotmap.RIGHT_FOLLOWER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushed)
        
        # shooter = rev.CANSparkMax(robotmap.SHOOTER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        # self.shooter = Shooter(shooter)
        
        #gyros
        self.ahrs = AHRS.create_spi()
        # self.navx = navx.AHRS.create_i2c()
        self.aimer = Aimer(self.ahrs)
        self.aimer.reset()
        
        shooter = rev.CANSparkMax(robotmap.SHOOTER_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.shooter = Shooter(shooter)
        
        self.left_motor_1.setClosedLoopRampRate(1.0)
        # self.left_motor_2.setClosedLoopRampRate(1.0)
        self.left_motor_3.setClosedLoopRampRate(1.0)

        self.right_motor_1.setClosedLoopRampRate(1.0)
        # self.right_motor_2.setClosedLoopRampRate(1.0)
        self.right_motor_3.setClosedLoopRampRate(1.0)
        #self.shooter.setClosedLoopRampRate(1.0)
        
        self.left_side = wpilib.SpeedControllerGroup(self.left_motor_1, self.left_motor_3)
        self.right_side = wpilib.SpeedControllerGroup(self.right_motor_1, self.right_motor_3)
        
        # Drivetrain
        self.drivetrain = wpilib.drive.DifferentialDrive(self.left_side, self.right_side)

        # Climber
        # assuming this is a Neo; otherwise it may not be brushless
        self.right_winch = rev.CANSparkMax(robotmap.WINCH_RIGHT_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.left_winch = rev.CANSparkMax(robotmap.WINCH_LEFT_ID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        self.winch = wpilib.SpeedControllerGroup(self.right_winch, self.left_winch)

        self.right_piston = wpilib.DoubleSolenoid(0, wpilib.PneumaticsModuleType.CTREPCM, robotmap.SOLENOID_RIGHT_FORWARD_ID, robotmap.SOLENOID_RIGHT_REVERSE_ID)
        self.left_piston = wpilib.DoubleSolenoid(0, wpilib.PneumaticsModuleType.CTREPCM, robotmap.SOLENOID_LEFT_FORWARD_ID, robotmap.SOLENOID_LEFT_REVERSE_ID)

        self.piston = climber.SolenoidGroup([self.right_piston, self.left_piston])

        self.climber = climber.Climber(self.piston, self.winch)
        self.drive = ARCADE

        # Change these depending on the controller
        self.left_trigger_axis = 2 
        self.right_trigger_axis = 3
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

        self.climber.solenoids.set(wpilib.DoubleSolenoid.Value.kReverse)

    def teleopPeriodic(self):
        # print("starting teleop periodic")
        """
        Makes the shooter motor spin. Right trigger -> 1, left trigger -> -0.2, 
        x reduces the speed, y reduces the speed more, b reduces the speed even more, 
        a reduces the speed the most
        """
        """
        if self.operator.getRawAxis(self.right_trigger_axis) > 0.95:
            # print("got trigger")
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

        else: # self.drive == SWERVE
            # Panik
            return
            
        '''
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
        '''
        
        # self.climber.solenoids.get()
        if self.operator.getXButtonPressed():
            self.climber.solenoids.toggle()
        if self.operator.getBButtonPressed():
            self.right_piston.toggle()

        self.climber.setWinch(self.deadzone(self.operator.getLeftY(), robotmap.deadzone))
        print(self.climber.winch.get())

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
        elif val < 0:
            x = (abs(val) - deadzone)/(1-deadzone)
            return -x
        else:
            x = (val - deadzone)/(1-deadzone)
            return x


if __name__ == "__main__":
    wpilib.run(MyRobot)
