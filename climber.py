from wpilib import DoubleSolenoid

# shorthands
kOff = DoubleSolenoid.Value.kOff
kForward = DoubleSolenoid.Value.kForward
kReverse = DoubleSolenoid.Value.kReverse


class Climber:
    """
    takes an array 'solenoids' of DoubleSolenoid objects, and a winch speed controller group
    """
    def __init__(self, solenoidGroup, winch, extendSpeed, retractSpeed):
        self.solenoids = solenoidGroup
        self.winch = winch
        self.climbstep = 0
        # Actions to Climb
        self.climbActions = [[self.winchExtend, 0.5], [self.pistonForward, 0.5], [self.winchExtend, 2.5],
                             [self.pistonReverse, 0.5], [self.winchRetract, 3.0], [self.winchOff, 0.1]]
        self.extendSpeed = extendSpeed
        self.retractSpeed = retractSpeed

    def climberOff(self):
         self.winch.set(0.0)  
         self.solenoids.set(kOff) 

    def setWinch(self, x): # positive corresponds to extend, negative corresponds to retract
        print("setWinch = ", x)
        if x >= 0: 
            self.winch.set(self.extendSpeed*x)
        else:
            self.winch.set(self.retractSpeed*x)

    def pistonForward(self):
        self.solenoids.set(kForward)

    def pistonReverse(self):
        self.solenoids.set(kReverse)

    def winchExtend(self):
        self.winch.set(self.extendSpeed)

    def winchRetract(self):
        self.winch.set(-self.retractSpeed)

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
    
    def get(self):  # value will be Forward, Reverse, or Off
        for s in self.solenoids:
            print(s.get())
    
    def toggle(self):
        for s in self.solenoids:
            s.toggle()


class WinchGroup:
    def __init__(self, right_winch, left_winch, right_limit, left_limit, cable_wrapped):
        self.right_winch = right_winch
        self.left_winch = left_winch

        self.right_limit = right_limit
        self.left_limit = left_limit
        self.cable_wrapped = cable_wrapped

        if self.cable_wrapped == 'UNDER':
            self.winch_mod = 1
        else: 
            self.winch_mod = -1

        # NEOs go counterclockwise when viewed facing 
        # the rotaty part and passed positive value
        # by default

        # so pulling the stick down leads to negative
        # value for setWinch which  if wrapped under
        # makes right winch spin in negative (clockwise)
        # which draws the cable down

    def set(self, speed):
        speed = speed*self.winch_mod

        # meaning right motor not allowed to spin clockwise (negative) more
        # assuming cable wrapped under
        
        if self.atRightLimit() or self.atLeftLimit():
            if speed < 0:
                speed = 0

        self.right_winch.set(speed)
        self.left_winch.set(-speed)

    def atRightLimit(self):
        return self.right_limit.get()
    
    def atLeftLimit(self):
        return self.left_limit.get()
