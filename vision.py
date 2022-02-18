import photonvision


class Vision:
    def __init__(self):
        self.camera = photonvision.PhotonCamera('photonvision')

    def get_yaw_degrees(self):
        result = self.camera.getLatestResult()
        if result.hasTargets():
            return result.getBestTarget().getYaw()
        return None
