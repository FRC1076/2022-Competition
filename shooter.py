class Shooter:
    def __init__(self, motor_controller, shooterRPM): # takes a SPARK MAX motorcontroller
        self.controller = motor_controller
        # self.controller.setClosedLoopRampRate(1.0)
        self.encoder = self.controller.getEncoder()
        self.pidController = self.controller.getPIDController()
        self.shooterRPM = shooterRPM
        
    def set(self, speed):
        self.controller.set(speed)

    def get(self):
        return self.controller.get()

    def getShooterRPM(self):
        return self.get()
        
    def setPIDController(self):
        kP = 0.00001
        kI = 0.00000
        kD = 0.00000
        kMaxOutput = 0
        kMinOutput = -1

        self.pidController.setP(kP)
        self.pidController.setI(kI)
        self.pidController.setD(kD)
        self.pidController.setOutputRange(kMinOutput, kMaxOutput)
