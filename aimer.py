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
    def __init__(self, gyro):
        self.gyro = gyro
        turnController = wpimath.controller.PIDController(kP, kI, kD)
        turnController.setTolerance(kToleranceDegrees)
        turnController.enableContinuousInput(-180.0, 180.0)
        self.turnController = turnController
    #    setRotateToAngleRate = 0

    def reset(self):
        self.gyro.reset()
    
    #def setAim(self, setPoint):
    #    self.setPoint = setPoint
    #    self.turnController.setSetPoint(self.gyro.getAngle() + self.setPoint())
    
    def getYaw(self):
        self.yaw = self.gyro.getYaw()
        return(self.yaw)

    #def getAngle(self):
    ##    self.ag = abs(self.gyro.getAngle()) % 360
        return self.ag

    def calculate(self, m):
        return(self.turnController.calculate(m))
    
    #def pidWrite(self, output):
    #    self.setRotateToAngleRate = output

    def calcRotationCoordinates(self, theta):
        angle = self.getYaw()
        #rotationRate = self.turnController.calculate(angle, theta)
        #return(rotationRate, 0)
        
        diff = abs(angle - theta)
        correctionFactor = (diff / 10.0)
        if (correctionFactor > 1.0):
            correctionFactor = 1.0
        
        if (diff > 1):
            if (theta > 0):
                return ((-0.6 * correctionFactor), 0)
            else:
                return ((0.6 * correctionFactor), 0)
        else:
            return (0, 0)
        

    def calculateTheta(self, x, y):
        y = -y
        theta = 0.0
        absY = abs(float(y))
        absX = abs(float(x))
        if(x == 0) and (y >= 0):
            theta = 0.0
        elif(x == 0) and (y <= 0):
            theta = 179.9
        elif(x > 0) and (y >= 0):
            theta = 90 - (math.atan(absY/absX)*180/math.pi)
        elif(x < 0) and (y >= 0):
            theta = -(90 - (math.atan(absY/absX)*180/math.pi))
        elif(x > 0) and (y < 0):
            theta = 90 + (90 - (math.atan(absY/absX)*180/math.pi))
        elif(x < 0) and (y < 0):
            theta = -(90 + (math.atan(absY/absX)*180/math.pi))
        else:
            theta = 0.0
            #print("unknown coordinates (", x, ", ", y, ")")
        return(theta)