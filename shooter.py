import wpilib


class Shooter:
    def __init__(self, motor_controller):
        self.motor = motor_controller

    def set(self, speed):
        self.motor.set(speed)