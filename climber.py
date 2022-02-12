from wpilib import DoubleSolenoid

# shorthands
kOff = DoubleSolenoid.Value.kOff
kForward = DoubleSolenoid.Value.kForward
kReverse = DoubleSolenoid.Value.kReverse


class Climber:
    """
    takes an array 'solenoids' of DoubleSolenoid objects, and a winch speed controller group
    """
    def __init__(self, solenoidGroup, winch):
        self.solenoids = solenoidGroup
        self.winch = winch
        self.climbstep = 0
        # Actions to Climb
        self.climbActions = [[self.winchForward, 0.5], [self.pistonForward, 0.5], [self.winchForward, 2.5],
                             [self.pistonReverse, 0.5], [self.winchReverse, 3.0], [self.winchOff, 0.1]]

    def climberOff(self):
         self.winch.set(0.0)  
         self.solenoids.set(kOff) 

    def setWinch(self, x):
        self.winch.set(x)

    def pistonForward(self):
        self.solenoids.set(kForward)

    def pistonReverse(self):
        self.solenoids.set(kReverse)

    def winchForward(self):
        self.winch.set(0.7)

    def winchReverse(self):
        self.winch.set(-0.7)

    def winchOff(self):
        self.winch.set(0.0)

    # Climbing
    def resetClimb(self):
        self.climbstep = 0

    def nextStep(self):
        self.climbstep = (self.climbstep + 1) % len(self.climbActions)

    def stepAction(self):
        self.climbActions[self.climbstep][0]()


class SolenoidGroup:
    def __init__(self, solenoids):
        self.solenoids = solenoids
        self.set(kOff)
    
    def set(self, value):  # value will be Forward, Reverse, or Off
        for s in self.solenoids:
            s.set(value)
    
    def toggle(self):
        for s in self.solenoids:
            s.toggle()
