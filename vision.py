import photonvision
import math


class Vision:
    def __init__(self, targetHeight, targetRadius, shooterHeight, shooterOffset, cameraHeight, cameraPitch):
        self.result = None

        self.camera = photonvision.PhotonCamera('mmal_service_16.1')
        self.camera.setDriverMode(False)
        self.camera.setPipelineIndex(0)
        self.latestResult = None

        self.yawlog = []
        self.pitchlog = []

        # everything is in feet
        self.targetHeight = targetHeight # 8.5
        self.targetRadius = targetRadius # 2

        # needs to be measured - position of the shooter is the center of mass of the ball as it leaves shooter
        self.shooterHeight = shooterHeight # 3.5
        self.shooterOffset = shooterOffset # 1  # horizontal offset of shooter from camera

        # needs to be measured
        self.cameraHeight = cameraHeight # 4

        # in degrees, subject to change
        #self.shooter_angle = 60
        #self.camera_angle = 0
        self.cameraPitch = cameraPitch # 0

        self.pitch = None
        self.yaw = None

        # lookup table for relationship btw shooter velocity & power

        self.v_p_table = {}

    def getPitchDegrees(self):
        return self.pitch

    def getYawDegrees(self):
        return self.yaw

    def getSmoothYaw(self):
        if self.result.hasTargets():
                
            sortedlog = sorted(self.yawlog)
            
            try:
                if len(sortedlog) == 1:
                    return sortedlog[0]
                else:
                    return sortedlog[1]
            except IndexError:
                pass
        else: 
            return None

    def getSmoothPitch(self):
        if self.result.hasTargets():
                
            sortedlog = sorted(self.pitchlog)
            
            try:
                if len(sortedlog) == 1:
                    return sortedlog[0]
                else:
                    return sortedlog[1]
            except IndexError:
                pass
        else: 
            return None

    def getDistanceFeet(self):  # the pitch is with respect to the ground
        if self.result.hasTargets():
            pitch = (math.pi / 180) * (self.getSmoothPitch() + self.cameraPitch)
            dist = (self.targetHeight - self.cameraHeight) / math.tan(pitch)
            return dist - self.shooterOffset + self.targetRadius
        else:
            return None 

    def calculateVelocity(self, angle):  # returns ft/s given shooter angle
        if self.result.hasTargets:
            x = self.getDistanceFeet()
            y = self.targetHeight - self.shooterHeight
            try:
                return math.sqrt(
                    (-16 * x ** 2) / ((y * (math.cos(angle)) ** 2) - (x * math.sin(angle) * math.cos(angle)))
                )
            
            except ValueError:
                return None
        else:
            return None
    
    def calculateAngle(self, velocity): #returns degrees given shooter velocity
        if self.result.hasTargets():

            v = velocity
            x = self.getDistanceFeet()
            if x is None:
                return None
            y = self.targetHeight - self.shooterHeight

            a = 16*(x**2 / v**2) # constant to make it look nicer
            try:
                return math.atan(
                    ( x + math.sqrt( x**2 - 4*(y + a)*a) ) / ( 2*a )    
                )
            except ValueError:
                return None
        else:
            return None

    def getNumTargets(self):
        if (self.latestResult):
            return(len(result.getTargets()))
        else:
            return 0

    def hasTargets(self):
        if (self.latestResult):
            return(self.latestResult.hasTargets())
        else:
            return(False)

    def getLatestResult(self):
        self.result = self.camera.getLatestResult()
        self.latestResult = self.result
        targets = self.result.getTargets()
        print("hasTargets = ", self.result.hasTargets())
        self.camera.takeOutputSnapshot()

        if self.result.hasTargets():
            self.pitch = sum([t.getPitch() * t.getArea() for t in targets]) / sum([t.getArea() for t in targets])
            self.yaw = sum([t.getYaw() * t.getArea() for t in targets]) / sum([t.getArea() for t in targets])

            self.pitchlog.append(self.pitch)
            if len(self.pitchlog) > 3:
                self.pitchlog.pop(0)

            self.yawlog.append(self.yaw)
            if len(self.yawlog) > 3:
                self.yawlog.pop(0)

            
        #distance = self.camera.getDistanceFeet()
        print("yaw = ", self.yaw, " pitch = ", self.pitch, " distance = ", self.getDistanceFeet())
        return targets