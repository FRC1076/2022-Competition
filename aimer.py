from navx import AHRS
import wpilib
import math
import wpimath.controller

kP = 0.1
kI = 0.00
kD = 0.0
kF = 0.0
kToleranceDegrees = 2.0


class Aimer:
    def __init__(self, gyro, rotationSpeed, accuracyDegrees):
        self.gyro = gyro
        turnController = wpimath.controller.PIDController(kP, kI, kD)
        turnController.setTolerance(kToleranceDegrees)
        turnController.enableContinuousInput(-180.0, 180.0)
        self.rotationSpeed = rotationSpeed
        self.accuracyDegrees = accuracyDegrees
        self.turnController = turnController
        self.theta = None

    #    setRotateToAngleRate = 0

    def reset(self):
        self.gyro.reset()

    # def setAim(self, setPoint):
    #    self.setPoint = setPoint
    #    self.turnController.setSetPoint(self.gyro.getAngle() + self.setPoint())

    def getTheta(self):
        return(self.theta)
    
    def setTheta(self, theta):
        self.theta = theta

    def getYaw(self):
        self.yaw = self.gyro.getYaw()
        return (self.yaw)

        # def getAngle(self):
        ##    self.ag = abs(self.gyro.getAngle()) % 360
        #return self.ag

    def calculate(self, m):
        return (self.turnController.calculate(m))

    # def pidWrite(self, output):
    #    self.setRotateToAngleRate = output

    def calcDiff(self, theta):
        if(not theta):
            return None
        angle = self.getYaw()
        return(abs(angle - theta))
    
    def getInRange(self, diff):
        if(diff > self.accuracyDegrees):
            return False
        else:
            return True
    
    def calculateDriveSpeeds(self, theta):
        angle = self.getYaw()
        # rotationRate = self.turnController.calculate(angle, theta)
        # return(rotationRate, 0)

        diff = self.calcDiff(theta)
        correctionFactor = (diff / 10.0)
        if correctionFactor > 1.0:
            correctionFactor = 1.0

        if (self.getInRange(diff)):
            print("diff <= ", self.accuracyDegrees)
            return (0, 0)
        else:
            if theta > 0:
                return (-self.rotationSpeed * correctionFactor), 0
            else:
                return (self.rotationSpeed * correctionFactor), 0
            
    def calculateTheta(self, x, y):
        y = -y
        theta = 0.0
        absY = abs(float(y))
        absX = abs(float(x))
        if (x == 0) and (y >= 0):
            theta = 0.0
        elif (x == 0) and (y <= 0):
            theta = 179.9
        elif (x > 0) and (y >= 0):
            theta = 90 - (math.atan(absY / absX) * 180 / math.pi)
        elif (x < 0) and (y >= 0):
            theta = -(90 - (math.atan(absY / absX) * 180 / math.pi))
        elif (x > 0) and (y < 0):
            theta = 90 + (90 - (math.atan(absY / absX) * 180 / math.pi))
        elif (x < 0) and (y < 0):
            theta = -(90 + (math.atan(absY / absX) * 180 / math.pi))
        else:
            theta = 0.0
            # print("unknown coordinates (", x, ", ", y, ")")
        return theta
