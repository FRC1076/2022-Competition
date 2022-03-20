from navx import AHRS
import wpilib
import math
import wpimath.controller

kP = 0.1
kI = 0.00
kD = 0.0
kF = 0.0
kToleranceDegrees = 1.5


class Aimer:
    def __init__(self, gyro, rotationSpeed, accuracyDegrees):
        self.gyro = gyro
        self.rotationSpeed = rotationSpeed
        self.accuracyDegrees = accuracyDegrees
        self.gyroSetPoint = None # target angle for the gyro to reach
        self.error = None # current offset from the target angle

        self.turnController = wpimath.controller.PIDController(kP, kI, kD)
        self.turnController.setTolerance(kToleranceDegrees)
        self.turnController.enableContinuousInput(-180.0, 180.0)
        self.turnController.setSetpoint(0) # we're calculating the error in the robot loop so this is fine

    def getError(self):
        return self.error

    def setError(self, error):
        self.error = error

    def reset(self):
        self.gyro.reset()

    def getYaw(self):
        return self.gyro.getYaw()

    def getAccumulatedYaw(self):
        return self.gyro.getAngle()

    def calcDiffModulated(self, theta):
        if(not theta):
            return None
        angle = self.getYaw() # in -180 to 180


        # make theta in -180 to 180 also
        theta = theta%360
        if 180<theta<=360:
            theta = theta - 360

        diff1 = abs(angle-theta)
        diff2 = 360 - abs(angle-theta)

        # we would like to return positive diff if theta is counterclockwise of angle, 
        # and negative diff if its clockwise of angle
        if diff1 < diff2: 
            return theta - angle

        else: # note if angle and theta have the same sign then diff1<diff2 
            # so in this case angle and theta have different signs. 
            if angle >= 0: # so theta <0 so theta is counterclockwise of angle
                return diff2
            else: 
                return -diff2

    def calcDiffAccumulated(self, theta): # Now theta can be any int
        return theta - self.gyro.getAngle()

    def PIDcalc(self, error):
        return self.turnController.calculate(error)
    
    def isInRange(self):
        return self.turnController.atSetpoint()
    
    '''
    def calculateDriveSpeeds(self, theta): # theta should be in -180 to 180  

        # rotationRate = self.turnController.calculate(angle, theta)
        # return(rotationRate, 0)

        diff = self.calcDiff(theta)


        correctionFactor = (diff / 10.0)
        if correctionFactor > 1.0:
            correctionFactor = 1.0

        #print("calculateDriveSpeeds: diff:", diff)
        if (self.getInRange(diff)):
            #print("diff <= ", self.accuracyDegrees)
            return (0, 0)
        else:
            if theta > 0:
                return (-self.rotationSpeed * correctionFactor), 0
            else:
                return (self.rotationSpeed * correctionFactor), 0
    '''
           
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
