import wpilib
import rev

class TiltShooter:
    def __init__(self, motor_controller, rotationsPer360, minDegrees, maxDegrees):
        self.motor = motor_controller
        self.motor.setClosedLoopRampRate(1.0)
        self.encoder = self.motor.getEncoder()
        self.rotationsPer360 = rotationsPer360
        self.encoder.setPosition(0) # Reset position of motor to zero
        self.minDegrees = minDegrees # Lower bound for tilt shooter
        self.maxDegrees = maxDegrees # Upper bound for tilt shooter
        self.targetDegrees = self.minDegrees # Starting position

    def set(self, speed):
        self.motor.set(speed)

    def get(self):
        return self.motor.get()

    def getTargetDegrees(self):
        return(self.targetDegrees)

    def setTargetDegrees(self, targetDegrees):
        if (targetDegrees < self.minDegrees):
            self.targetDegrees = self.minDegrees
        elif (targetDegrees > self.maxDegrees):
            self.targetDegrees = self.maxDegrees
        else:
            self.targetDegrees = targetDegrees

    def getMinDegrees(self):
        return(self.minDegrees)

    def getMaxDegrees(self):
        return(self.maxDegrees)

    def getRotations(self):
        return(self.encoder.getPosition())
    
    def getDegrees(self):
        return(self.rotationsToDegrees(self.getRotations()))

    def getVelocity(self):
        return(self.encoder.getVelocity())

    def rotationsToDegrees(self, rotations):
        return(rotations * 360 / self.rotationsPer360)

    def degreesToRotations(self, degrees):
        return(degrees * self.rotationsPer360 / 360)
