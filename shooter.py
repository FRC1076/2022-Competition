import wpilib

class Shooter:
    def __init__(self, motor_controller):
        self.motor = motor_controller
        self.motor.setClosedLoopRampRate(1.0)
        self.encoder = self.getEncoder()

    def set(self, speed):
        self.motor.set(speed)

    def get(self):
        return self.motor.get()
    
    def getCPR(self):
        return self.encoder.getCountsPerRevolution()
    
    def getRPM(self):
        return self.encoder.getVelocity()
    
    def setRPM(self, rpm):
        #I don't know if this function works as intended

        #Gets Native Output Units
        native_output = self.getRPM()/self.encoder.getVelocityConversionFactor()
        #With native_output, divide from RPM to get new Conversion Factor
        self.encoder.setVelocityConversionFactor(rpm/native_output)