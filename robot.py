import math
import time
import wpilib
import wpilib.drive
from shooter import Shooter
from wpilib import interfaces
import rev
import robotmap
import climber

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):

        # Create both xbox controlers
        self.driver = wpilib.XboxController(0)
        self.operator = wpilib.XboxController(1)

        self.LEFT_HAND = wpilib._wpilib.XboxController.Hand.kLeftHand
        self.RIGHT_HAND = wpilib._wpilib.XboxController.Hand.kRightHand

        # Motors
        
        self.left_motor_1 = rev.CANSparkMax(robotmap.LEFT_LEADER_ID, rev.MotorType.kBrushed)
        self.left_motor_2 = rev.CANSparkMax(robotmap.LEFT_MIDDLE_ID, rev.MotorType.kBrushed)
        self.left_motor_3  = rev.CANSparkMax(robotmap.LEFT_FOLLOWER_ID, rev.MotorType.kBrushed)
        self.right_motor_1 = rev.CANSparkMax(robotmap.RIGHT_LEADER_ID, rev.MotorType.kBrushed)
        self.right_motor_2 = rev.CANSparkMax(robotmap.RIGHT_MIDDLE_ID, rev.MotorType.kBrushed)
        self.right_motor_3 = rev.CANSparkMax(robotmap.RIGHT_FOLLOWER_ID, rev.MotorType.kBrushed)
        

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
        self.drive = TANK

        # Climber


        self.right_winch = rev.CANSparkMax(robotmap.WINCH_RIGHT_ID, rev.MotorType.kBrushless) # assuming this is a Neo; otherwise it may not be brushless
        self.left_winch = rev.CANSparkMax(robotmap.WINCH_LEFT_ID, rev.MotorType.kBrushless)

        self.winch = wpilib.SpeedControllerGroup(self.right_winch, self.left_winch)

        self.right_piston = wpilib.DoubleSolenoid(0, wpilib.PneumaticsModuleType.CTREPCM, robotmap.SOLENOID_RIGHT_FORWARD_ID, robotmap.SOLENOID_RIGHT_REVERSE_ID)
        self.left_piston = wpilib.DoubleSolenoid(0, wpilib.PneumaticsModuleType.CTREPCM, robotmap.SOLENOID_LEFT_FORWARD_ID, robotmap.SOLENOID_LEFT_REVERSE_ID)

        self.piston = climber.SolenoidGroup(self.right_piston, self.left_piston)
        # Change these depending on the controller
        self.left_trigger_axis = 2 
        self.right_trigger_axis = 5
        print("running!")


    def robotPeriodic(self):
        pass

    def teleopInit(self):
        self.shooter_mod = 1
        self.running = 0

    def teleopPeriodic(self):
        #print("starting teleop periodic")
        """
        Makes the shooter motor spin. Right trigger -> 1, left trigger -> -0.2, 
        x reduces the speed, y reduces the speed more, b reduces the speed even more, 
        a reduces the speed the most
        """
        if self.operator.getRawAxis(self.right_trigger_axis) > 0.95:
            print("got trigger")
            self.running = 1
        elif self.operator.getRawAxis(self.left_trigger_axis) > 0.95:
            self.running = -0.2
        else:
            self.running = 0 

        if self.operator.getAButton():
            print("got A button!")
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

            leftspeed = self.driver.getY(LEFT_HAND)
            rightspeed = self.driver.getY(RIGHT_HAND)
            #leftspeed = 0.5
            #rightspeed = 0.5
            #Invoke deadzone on speed.
            leftspeed = 0.80 * self.deadzone(leftspeed, robotmap.deadzone)
            rightspeed = 0.80 * self.deadzone(rightspeed, robotmap.deadzone)
            print("Right Speed: ", rightspeed)
            print("Left Speed: ", leftspeed)
            #Invoke Tank Drive
            self.drivetrain.tankDrive(leftspeed, rightspeed)

        #ARCADE DRIVE
        elif (self.drive == ARCADE):

            #Get left (forward) joystick value
            forward = self.driver.getY(RIGHT_HAND) 
            forward = 0.80 * self.deadzone(forward, robotmap.deadzone)

            #Get right (rotation) joystack Value
            rotation_value = -0.8 * self.driver.getX(LEFT_HAND)
        
            #Invoke Arcade Drive
            self.drivetrain.arcadeDrive(forward, rotation_value)

        else: #self.drive == SWERVE
            #Panic
            return

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

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
