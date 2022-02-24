class Controller:
    def __init__(self, xboxController: None, deadzone: 0.0,
                 left_trigger_axis: 2, right_trigger_axis: 3):
        self.xboxController = xboxController
        self.deadzone = deadzone
        self.left_trigger_axis = left_trigger_axis
        self.right_trigger_axis = right_trigger_axis
