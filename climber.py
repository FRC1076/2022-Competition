import wpilib
from wpilib._wpilib.DoubleSolenoid.Value import kOff
from wpilib.DoubleSolenoid.Value import kForward 
from wpilib.DoubleSolenoid.Value import kReverse 
from robotmap import SOLENOID_LEFT_REVERSE_ID


class Climber: 
    '''
    takes an array 'solenoids'of DoubleSolenoid objects, and a winch speed controller group
    '''
    def __init__(self, solenoidGroup, winch):
        self.solenoids = solenoidGroup
        self.winch = winch
        pass

    #Actions to Climb

    def pistonforward(self):
        self.solenoids.set(kForward)
    
    def winchforward(self):
        self.winch.set(1.0)
    
    def pistonback(self):
        self.solenoids.set(kReverse)

    def winchback(self):
        self.winch.set(-1.0)

    climbActions = [pistonforward, winchforward, pistonback, winchback]

    #Climbing
    def startclimb(self):
        self.climbstep = 0

    def nextclimb(self):
        self.climbActions[self.climbstep]()
        self.climbstep += 1

class SolenoidGroup:
    def __init__(self, solenoids):
        self.solenoids = solenoids
        self.set(kOff)
    
    def set(self, value): #value will be Forward, Reverse, or Off
        for s in self.solenoids:
            s.set(value)
    



