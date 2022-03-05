import wpilib
import rev

class TiltShooter:
    def __init__(self, motor_controller, rotationsPer360, minDegrees, maxDegrees, bufferDegrees, targetSpeed):
        self.motor = motor_controller
        self.motor.setClosedLoopRampRate(1.0)
        self.encoder = self.motor.getEncoder()
        self.rotationsPer360 = rotationsPer360
        self.encoder.setPosition(0) # Reset position of motor to zero
        self.minDegrees = minDegrees # Lower bound for tilt shooter
        self.maxDegrees = maxDegrees # Upper bound for tilt shooter
        self.targetDegrees = self.minDegrees # Starting position
        #self.manualTiltShooter = True
        self.bufferDegrees = bufferDegrees
        self.targetSpeed = targetSpeed

    def resetPosition(self):
        self.encoder.setPosition(0)
        
    def setSpeed(self, speed):
        self.motor.set(speed)

    def getSpeed(self):
        return self.motor.get()

    def getTargetSpeed(self):
        return(self.targetSpeed)

    def getTargetDegrees(self):
        return(self.targetDegrees)

    def setTargetDegrees(self, targetDegrees):
        if (targetDegrees < self.minDegrees):
            self.targetDegrees = self.minDegrees
        elif (targetDegrees > self.maxDegrees):
            self.targetDegrees = self.maxDegrees
        else:
            self.targetDegrees = targetDegrees
        return(self.targetDegrees)

    def getMinDegrees(self):
        return(self.minDegrees)

    def getMaxDegrees(self):
        return(self.maxDegrees)

    def getBufferDegrees(self):
        return(self.bufferDegrees)

    '''
    def getManualTiltShooter(self):
        return(self.manualTiltShooter)
    
    def setManualTiltShooter(self, manualTiltShooter):
        self.manualTiltShooter = manualTiltShooter
        return(self.manualTiltShooter)
    '''

    def getRotations(self):
        return(self.encoder.getPosition())
    
    def getDegrees(self):
        return(self.rotationsToDegrees(self.getRotations()))

    def getNearTarget(self):
        if((self.getDegrees() > (self.getTargetDegrees() - self.bufferDegrees)) and (self.getDegrees() > (self.getTargetDegrees() - self.bufferDegrees))):
            return True
        else:
            return False
    
    def getVelocity(self):
        return(self.encoder.getVelocity())

    def rotationsToDegrees(self, rotations):
        return(rotations * 360 / self.rotationsPer360)

    def degreesToRotations(self, degrees):
        return(degrees * self.rotationsPer360 / 360)
