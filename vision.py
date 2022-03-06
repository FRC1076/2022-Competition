import photonvision
import math


class Vision:
    def __init__(self, targetHeight, targetRadius, shooterHeight, shooterOffset, cameraHeight, cameraPitch):
        self.camera = photonvision.PhotonCamera('mmal_service_16.1')
        self.camera.setDriverMode(False)
        self.camera.setPipelineIndex(0)

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
        sortedlog = sorted(self.yawlog)
        try:
            return sortedlog[1]
        except IndexError:
            pass

    def getSMoothPitch(self):
        sortedlog = sorted(self.pitchlog)
        try:
            return sortedlog[1]
        except IndexError:
            pass

    def getDistanceFeet(self):  # the pitch is with respect to the ground
        pitch = (math.pi / 180) * (self.SmoothPitch() + self.cameraPitch)
        dist = (self.targetHeight - self.cameraHeight) / math.tan(pitch)
        return dist - self.shooterOffset + self.targetRadius

    def calculateVelocity(self, angle):  # returns ft/s given shooter angle
        x = self.getDistanceFeet()
        y = self.targetHeight - self.shooterHeight

        return math.sqrt(
            (-16 * x ** 2) / ((y * (math.cos(angle)) ** 2) - (x * math.sin(angle) * math.cos(angle)))
        )
    
    def calculateAngle(self, velocity): #returns degrees given shooter velocity
        v = velocity
        x = self.getDistanceFeet()
        y = self.targetHeight - self.shooterHeight

        a = 16*(x**2 / v**2) # constant to make it look nicer

        return math.atan(
            ( x + math.sqrt( x**2 - 4*(y + a)*a) ) / ( 2*a )    
        )

    def getLatestResult(self):
        result = self.camera.getLatestResult()
        targets = result.getTargets()
        #print(result.hasTargets())
        self.camera.takeOutputSnapshot()

        if len(result.getTargets()) > 3:
            self.pitch = sum([t.getPitch() * t.getArea() for t in targets]) / sum([t.getArea() for t in targets])
            self.yaw = sum([t.getYaw() * t.getArea() for t in targets]) / sum([t.getArea() for t in targets])

            self.pitchlog.append(self.pitch)
            if len(self.pitchlog) > 3:
                self.pitchlog.pop(0)

            self.yawlog.append(self.yaw)
            if len(self.yawlog) > 3:
                self.yawlog.pop(0)

            return targets