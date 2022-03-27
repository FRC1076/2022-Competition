import wpilib
import rev

class Drivetrain:
    def __init__(self, motor_controllers, gear_ratio, wheel_circumference, countsPerRevolution, left_encoder_motor, right_encoder_motor):
        self.motors = motor_controllers
        #self.motors.setClosedLoopRampRate(1.0)
        self.gearRatio = gear_ratio
        self.wheelCircumference = wheel_circumference
        self.countsPerRevolution = countsPerRevolution
        self.leftEncoderMotor = left_encoder_motor
        self.rightEncoderMotor = right_encoder_motor
        if self.leftEncoderMotor:
            self.leftEncoder = self.leftEncoderMotor.getAlternateEncoder(self.countsPerRevolution)
            self.leftEncoder.setPosition(0) # Reset position of motor to zero
        if self.rightEncoderMotor:
            self.rightEncoder = self.rightEncoderMotor.getAlternateEncoder(self.countsPerRevolution)
            self.rightEncoder.setPosition(0) # Reset position of motor to zero
        self.resetPosition()

    def resetPosition(self):
        self.leftResetPosition()
        self.rightResetPosition()

    def leftResetPosition(self):
        if self.leftEncoderMotor and self.leftEncoder:
            self.leftEncoder.setPosition(0)
    
    def rightResetPosition(self):
        if self.rightEncoderMotor and self.rightEncoder:
            self.rightEncoder.setPosition(0)
        
    def getLeftRotations(self):
        if self.leftEncoderMotor and self.leftEncoder:
            return(self.leftEncoder.getPosition())
        else:
            return None
    
    def getRightRotations(self):
        if self.rightEncoderMotor and self.rightEncoder:
            return(self.rightEncoder.getPosition())
        else:
            return None
    
    def getLeftInches(self):
        if self.getLeftRotations():
            print("Get Left Inches: ", (self.rotationsToInches(self.getLeftRotations())))
            return(self.rotationsToInches(self.getLeftRotations()))
        else:
            return 0

    def getRightInches(self):
        if self.getRightRotations():
            print("Get Right Inches: ", (self.rotationsToInches(self.getRightRotations())))
            return(self.rotationsToInches(self.getRightRotations()))
        else:
            return 0

    def rotationsToInches(self, rotations):
        #print("Left: ", self.getLeftRotations(), " Right: ", self.getRightRotations())
        #return(abs(rotations * (12 / 55) * (20 / 54) * self.wheelCircumference))
        #return(abs(rotations * 69.44 * self.wheelCircumference))
        return(abs(rotations * 1309))
        # 1309 = 15 * 12 / 0.1375
        # OR (55 / 12) * (54 / 20) = 12.375

