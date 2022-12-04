import wpilib
import rev

class Hooks:
    def __init__(self, motors):
        #front, back, left, right
        self.motors = motors
    
    def set_speed(self, speed):
        for motor in self.motors:
            motor.set(speed)
    
    def get_speed(self):
        self.motor.get()