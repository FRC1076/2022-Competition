import wpilib

class Feeder:
    def __init(self, motor_controller):
        self.feeder = motor_controller
        self.motor.setClosedLoopRampRate(1.0)

    def setFeeder(self, speed)
        self.motor.set(speed)
    