from wpilib import DoubleSolenoid

kOff = DoubleSolenoid.Value.kOff
kForward = DoubleSolenoid.Value.kForward
kReverse = DoubleSolenoid.Value.kReverse


# Simplified version of the Climber class
class Intake:
    def __init__(self, doubleSolenoid, motor, motorSpeed=0.5):
        self.solenoid = doubleSolenoid
        self.motor = motor
        self.motorSpeed = motorSpeed

        self.solenoid.set(kOff)

    def toggle(self):
        """
        Toggles the state of the piston between kForward and kReverse.
        If the state is kOff, it defaults to kReverse
        """
        if self.solenoid.get() == kForward:
            self.solenoid.set(kReverse)
        elif self.solenoid.get() == kReverse or self.solenoid.get() == kOff:
            self.solenoid.set(kForward)

    def extend(self):
        self.solenoid.set(kForward)

    def retract(self):
        self.solenoid.set(kReverse)

    def motorOn(self):
        self.motor.set(self.motorSpeed)

    def motorOff(self):
        self.motor.set(0.0)
