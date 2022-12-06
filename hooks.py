import wpilib
import rev

class Hooks:
    def __init__(self, motors):
        #front, back, left, right
        self.motors = motors
        self.modules = [HookModule(motors[0]), HookModule(motors[1]), HookModule(motors[2]), HookModule(motors[3])]

    def change_front(self):
        self.modules[0].change_state()
    
    def change_back(self):
        self.modules[1].change_state()

    def change_left(self):
        self.modules[2].change_state()
    
    def change_right(self):
        self.modules[3].change_state()
    
    


class HookModule:
    def __init__(self, motor):
        self.motor = motor
        #0:raised, 1:raising, 2:lowered, 3:lowering
        self.state = 0

    def change_state(self):
        self.state += 1
        if self.state == 4:
            self.state = 0

        #if raised or lowered
        if self.state == 0 or self.state == 2:
            self.set(0)
        
        #if raising
        if self.state == 1:
            self.set(-0.1)
        
        #if lowering
        if self.state == 3:
            self.state(0.1)
    
    def get_state(self):
        return self.state