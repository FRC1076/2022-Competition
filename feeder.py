import wpilib

class Feeder:
    def __init__(self, motor_controller, feederSpeed):
        self.motor = motor_controller
        self.feederSpeed = feederSpeed
        self.motor.setClosedLoopRampRate(1.0)
        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(0) # Reset position of motor to zero

    def setFeeder(self, speed):
        print("Setting feeder speed: ", speed)
        self.motor.set(speed)

    def setPosition(self, position):
        pass
    
    def getPosition(self):
        return(self.encoder.getPosition())

    def hasFired(self):
        if(self.getPosition() > 10):
            return True
        else:
            return False

    def reset(self):
        self.setPosition(0)
    