import wpilib

class Shooter:
    def __init__(self, motor_controller, shooterRPM, shooterMaxRPM, shooterMinRPM): # takes a SPARK MAX motorcontroller
        self.controller = motor_controller
        # self.controller.setClosedLoopRampRate(1.0)
        self.encoder = self.controller.getEncoder()
        self.pidController = self.controller.getPIDController()
        self.configShooterRPM = shooterRPM
        self.storedShooterRPM = shooterRPM
        self.shooterMaxRPM = shooterMaxRPM
        self.shooterMinRPM = shooterMinRPM
        
    def setShooterVelocity(self, speed):
        self.controller.set(speed)

    def getShooterVelocity(self):
        return self.controller.get()

    def getConfigShooterRPM(self):
        return(self.configShooterRPM)

    def setStoredShooterRPM(self, storedRPM):
        if(storedRPM > self.getShooterMaxRPM()):
            self.storedShooterRPM = self.getShooterMaxRPM()
        elif(storedRPM < self.getShooterMinRPM()):
            self.storedShooterRPM = self.getShooterMinRPM()
        else:
            self.storedShooterRPM = storedRPM

    def getStoredShooterRPM(self):
        return self.storedShooterRPM
    
    def getShooterMaxRPM(self):
        return self.shooterMaxRPM

    def getShooterMinRPM(self):
        return self.shooterMinRPM
        
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
