import wpilib

class Feeder:
    def __init__(self, motor_controller):
        self.motor = motor_controller
        self.motor.setClosedLoopRampRate(1.0)

    def setFeeder(self, speed):
        self.motor.set(speed)
    