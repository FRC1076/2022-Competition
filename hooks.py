import wpilib
import rev
from robotconfig import robotconfig

class Hooks:
    def __init__(self, motors):
        #self.hookSpeed = 0.5
        config = robotconfig["HOOKS"]
        #front, back, left, right
        self.motors = motors
        self.modules = [HookModule(motors[0], config['FRONT_TOP_PORT'], config['FRONT_BOTTOM_PORT']), 
                            HookModule(motors[1], config['BACK_TOP_PORT'], config['BACK_BOTTOM_PORT']), 
                                HookModule(motors[2], config['LEFT_TOP_PORT'], config['LEFT_BOTTOM_PORT']), 
                                    HookModule(motors[3], config['RIGHT_TOP_PORT'], config['RIGHT_BOTTOM_PORT'])]

    def change_front(self):
        self.modules[0].change_state()
    
    def change_back(self):
        self.modules[1].change_state()

    def change_left(self):
        self.modules[2].change_state()
    
    def change_right(self):
        self.modules[3].change_state()
    
    #check if any limit switches are triggered
    def update(self):
        self.modules[0].update()
        self.modules[1].update()
        self.modules[2].update()
        self.modules[3].update()

class HookModule:
    #motor, top limit switch port number, bottom limit switch port number
    def __init__(self, motor, top_port, bottom_port):
        self.hookSpeed = 0.5
        self.motor = motor
        #0:raised, 1:raising, 2:lowered, 3:lowering
        self.state = 2
        #top switch
        self.top_switch = wpilib.DigitalInput(top_port)
        #bottom switch
        self.bottom_switch = wpilib.DigitalInput(bottom_port)
        #making sure that the motor is only stopped once...allows motor to turn other direction without stopping it
        self.trigger_once = False

    #change the state of hook when button is pressed
    def change_state(self):
        if self.state == 0 or self.state == 1:
            self.state = 3
        elif self.state == 2 or self.state == 3:
            self.state = 1

        #if raised or lowered
        if self.state == 0 or self.state == 2:
            self.motor.set(0)
        
        #if raising
        if self.state == 1:
            self.motor.set(-self.hookSpeed)
        
        #if lowering
        if self.state == 3:
            self.motor.set(self.hookSpeed)
    
    #set the state of hook
    def set_state(self, state):
        self.state = state
        #if raised or lowered
        if self.state == 0 or self.state == 2:
            self.motor.set(0)
        
        #if raising
        if self.state == 1:
            self.motor.set(-self.hookSpeed)
        
        #if lowering
        if self.state == 3:
            self.motor.set(self.hookSpeed)
    
    #get the state of hook
    def get_state(self):
        return self.state
    
    #check if any limit switch is triggered
    def update(self):
        if self.bottom_switch.get() == True and self.trigger_once == False:
            #MAKE SURE TO USE set_state() FUNCTION
            self.set_state(0)
            self.trigger_once = True

        if self.top_switch.get() == True and self.trigger_once == False:
            self.set_state(2)
            self.trigger_once = True

        if self.bottom_switch.get() == False and self.top_switch.get() == False:
            self.trigger_once = False

        print(self.state, self.top_switch.get(), self.bottom_switch.get())