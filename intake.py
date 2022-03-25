from wpilib import DoubleSolenoid

kOff = DoubleSolenoid.Value.kOff
kForward = DoubleSolenoid.Value.kForward
kReverse = DoubleSolenoid.Value.kReverse


# Simplified version of the Climber class
class Intake:
    def __init__(self, doubleSolenoid, motor, intakeMotorSpeed):
        self.solenoid = doubleSolenoid
        self.motor = motor
        self.intakeMotorSpeed = intakeMotorSpeed

        self.solenoid.set(kOff)
        print("MARK")

    def toggle(self):
        """
        Toggles the state of the piston between kForward and kReverse.
        If the state is kOff, it defaults to kReverse
        """
        print("Toggling Intake")
        if self.solenoid.get() == kForward:
            self.solenoid.set(kReverse)
        #elif (self.solenoid.get() == kReverse or self.solenoid.get() == kOff):
        else:
            self.solenoid.set(kForward)

    def extend(self):
        self.solenoid.set(kForward)

    def retract(self):
        self.solenoid.set(kReverse)
    
    def motorOn(self):
        self.motor.set(self.intakeMotorSpeed)

    def motorOff(self):
        self.motor.set(0.0)
    