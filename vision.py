import photonvision
import math


class Vision:
    def __init__(self):
        self.camera = photonvision.PhotonCamera('photonvision')
        self.yawlog = []
        self.pitchlog = []

        # everything is in feet
        self.target_height = 8.5
        self.target_radius = 2

        # needs to be measured - position of the shooter is the center of mass of the ball as it leaves shooter
        self.shooter_height = 3.5
        self.shooter_offset = 1  # horizontal offset of shooter from camera

        # needs to be measured
        self.camera_height = 4

        # in degrees, subject to change
        self.shooter_angle = 60
        self.camera_angle = 0
        self.camera_pitch = 0

        self.pitch = None
        self.yaw = None

    def get_pitch_degrees(self, target):
        return self.pitch

    def get_yaw_degrees(self, target):
        return self.yaw

    def get_smooth_yaw(self):
        sortedlog = sorted(self.yawlog)
        return sortedlog[1]

    def get_smooth_pitch(self):
        sortedlog = sorted(self.pitchlog)
        return sortedlog[1]

    def get_dist_ft(self, pitch):  # the pitch is with respect to the ground
        dist = (self.target_height - self.camera_height) / math.tan(pitch)
        return dist - self.shooter_offset + self.target_radius

    def calculate_velocity(self, target):  # returns ft/s
        pitch = (math.pi / 180) * (self.get_smooth_pitch() + self.camera_pitch)
        x = self.get_dist_ft(pitch)
        y = self.target_height - self.shooter_height

        return math.sqrt(
            (-16 * x ** 2) / ((y * (math.cos(pitch)) ** 2) - (x * math.sin(pitch) * math.cos(pitch)))
        )

    def get_latest_result(self):
        result = self.camera.getLatestResult()
        targets = result.getTargets()

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
