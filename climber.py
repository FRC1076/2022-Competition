import wpilib

class Climber:
    def __init__(self, motor_controller, DoubleSolenoid):
        self.piston = DoubleSolenoid

    #def extendWinch(self, extendAmount):


    #def retractWinch(self, retractAmount):

    
    def togglePiston(self):
        piston.togglePiston()
