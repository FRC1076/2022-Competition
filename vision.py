from unittest import result
import photonvision
import math


class Vision:
    def __init__(self):
        self.camera = photonvision.PhotonCamera('photonvision')
        self.yawlog = []

        self.rectwidth = 0.417 #everything is in feet
        self.target_height = 8.5
        self.camera_height = 4

    def get_pitch_degrees(self):
        
    def get_yaw_degrees(self):
        result = self.camera.getLatestResult()
        targets = result.getTargets()
        target = targets[len(targets)//2]
        
        if result.hasTargets():
            return target.getBestTarget().getYaw()
        self.yawlog.append()
        if len(self.yawlog) > 3:
            self.yawlog.pop(0)
        return None

    def get_smooth_yaw(self):
        sortedlog = sorted(self.yawlog)
        return sortedlog[1]

    def calc_floordistance(self):
        pitch = self.
        dist = 
        return dist * cos(pitch)

    def calc_dist(self):
        