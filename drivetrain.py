import wpilib
import rev

class Drivetrain:
    def __init__(self, motor_controller, rotationsPerInch, countsPerRevolution):
        self.motor = motor_controller
        self.motor.setClosedLoopRampRate(1.0)
        self.rotationsPerInch = rotationsPerInch
        self.countsPerRevolution = countsPerRevolution
        self.encoder = self.motor.getEncoder(kQuadrature, self.countsPerRevolution)
        self.encoder.setPosition(0) # Reset position of motor to zero
    
    def resetPosition(self):
        self.encoder.setPosition(0)
    
    def getRotations(self):
        return(self.encoder.getPosition())
    
    def getInches(self):
        return(self.rotationsToInches(self.getRotations()))

    def rotationsToInches(self, rotations):
        return(rotations * 12 / self.rotationsPerInch)

    def inchesToRotations(self, inches):
        return(inches * self.rotationsPerInch / 12)
