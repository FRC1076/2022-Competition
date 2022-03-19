import wpilib
import rev

class Drivetrain:
    def __init__(self, motorControllers, gearRatio, wheelCircumference, countsPerRevolution, leftEncoderMotor, rightEncoderMotor):
        self.motors = motorControllers
        #self.motors.setClosedLoopRampRate(1.0)
        self.gearRatio = gearRatio
        self.wheelCircumference = wheelCircumference
        self.countsPerRevolution = countsPerRevolution
        self.leftEncoderMotor = leftEncoderMotor
        self.rightEncoderMotor = rightEncoderMotor
        if self.leftEncoderMotor:
            self.leftEncoder = self.leftEncoderMotor.getAlternateEncoder(self.countsPerRevolution)
            self.leftEncoder.setPosition(0) # Reset position of motor to zero
        if self.rightEncoderMotor:
            self.rightEncoder = self.rightEncoderMotor.getAlternateEncoder(self.countsPerRevolution)
            self.rightEncoder.setPosition(0) # Reset position of motor to zero

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
            return(self.rotationsToInches(self.getLeftRotations()))
        else:
            return None

    def getRightInches(self):
        if self.getRightRotations():
            return(self.rotationsToInches(self.getRightRotations()))
        else:
            return None

    def rotationsToInches(self, rotations):
        return(rotations / self.gearRatio * self.wheelCircumference)

