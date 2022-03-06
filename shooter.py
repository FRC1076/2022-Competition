import wpilib

class Shooter:
    def __init__(self, motor_controller): # takes a SPARK MAX motorcontroller
        self.controller = motor_controller
        # self.controller.setClosedLoopRampRate(1.0)
        self.encoder = self.controller.getEncoder()
        self.pidController = self.controller.getPIDController()
        
    def set(self, speed):
        self.controller.set(speed)

    def get(self):
        return self.controller.get()

    def calcShooterSpeedChange(self): 
        pass
        
    def setPIDController(self):
        kP = 0.00055
        kI = 0.000001
        kD = 0.00005
        kMaxOutput = 1
        kMinOutput = -1

        self.pidController.setP(kP)
        self.pidController.setI(kI)
        self.pidController.setD(kD)
        self.pidController.setOutputRange(kMinOutput, kMaxOutput)
