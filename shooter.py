import wpilib

class Shooter:
    def __init__(self, motor_controller):
        self.motor = motor_controller
        self.motor.setClosedLoopRampRate(1.0)

    def set(self, speed):
        self.motor.set(speed)

    def get(self):
        return self.motor.get()
    