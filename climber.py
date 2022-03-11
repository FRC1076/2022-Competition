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

    def getWinch(self):
        return(self.winch.get())
    
    def setWinch(self, leftSpeedMod, rightSpeedMod):
        
        # positive corresponds to extend, negative corresponds to retract
        if (leftSpeedMod >= 0):
            leftSpeed = self.extendSpeed * leftSpeedMod
        else:
            leftSpeed = self.retractSpeed * leftSpeedMod
        
        if (rightSpeedMod >= 0):
            rightSpeed = self.extendSpeed * rightSpeedMod
        else: 
            rightSpeed = self.retractSpeed * rightSpeedMod

        self.winch.set(leftSpeed, rightSpeed)

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

    def get(self):
        return (self.right_winch.get(), self.left_winch.get())
    
    def set(self, left_speed, right_speed):
        
        # meaning right motor not allowed to spin clockwise (negative) more
        # assuming cable wrapped under
        
        leftSpeed = left_speed * self.winch_mod
        rightSpeed = right_speed * self.winch_mod
        
        #print("limits: ", self.atRightLimit(), self.atLeftLimit())
        if self.atRightLimit():
            if rightSpeed > 0:
                rightSpeed = 0
        if self.atLeftLimit():
            if leftSpeed > 0:
                leftSpeed = 0

        self.right_winch.set(rightSpeed)
        self.left_winch.set(-leftSpeed)

    def atRightLimit(self):
        return not(self.right_limit.get())
    
    def atLeftLimit(self):
        return not(self.left_limit.get())
