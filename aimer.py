from navx import AHRS
import wpilib
import math
import wpimath.controller

kP = 0.05
kI = 0.02
kD = 0.0
kF = 0.0
kToleranceDegrees = 2.0

class Aimer:
    def __init__(self, gyro):
        self.gyro = gyro
        turnController = wpimath.controller.PIDController(kP, kI, kD)
        turnController.setTolerance(kToleranceDegrees)
        turnController.enableContinuousInput(-180.0, 180.0)
        self.turnController = turnController
        setRotateToAngleRate = 0

    def reset(self):
        self.gyro.reset()
    
    def setAim(self, setPoint):
        self.setPoint = setPoint
        self.turnController.setSetPoint(self.gyro.getAngle() + self.setPoint())
    
    def getYaw(self):
        self.yaw = self.gyro.getYaw()
        return(self.yaw)

    def getAngle(self):
        self.ag = abs(self.gyro.getAngle()) % 360
        return self.ag

    def calculate(self, m):
        return(self.turnController.calculate(m))
    
    def pidWrite(self, output):
        self.setRotateToAngleRate = output