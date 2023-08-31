from enum import Enum

class InputType(Enum):
    ERROR = 0
    FOLLOW = 1
    INCREMENT = 2

class ControllerType(Enum):
    ERROR = 0
    CARTESIAN = 1
    JOINT = 2

class TeleopController():
    def __init__(self,
            input_type: InputType,
            controller_type: ControllerType):
            # input_to_rot_adjustment = Rotation.Quaternion(0, 0, 0, 1)): # Not doing input to rot adjustment now, as it gets confusing
        self.input_type = input_type
        self.controller_type = controller_type

        self.is_registered = False
        self.is_clutched = False
        self.is_enabled = False

    def register(self, output_callback):
        self.output_callback = output_callback
        self.is_registered = True

    def clutch(self):
        self.is_clutched = True

    def _unclutch(self):
        self.is_clutched = False

    # Enable/disable is for if you want your input to disable control of an output,
    # and input to enable to control another output
    # This way, input can also control various outputs simultaneously,
    # by creating multiple TeleopControllers with the same input.
    def _enable(self):
        self.is_enabled = True

    def disable(self):
        self.is_enabled = False

    def _update(self):
        if(not self.is_registered):
            print("Hey, need to call register. I'm not updating")
            return False
        if(self.is_clutched):
            return False
        if(not self.is_enabled):
            return False

        return True

    ## Event driven by input frequency. Call this in your input loop/callback
    def update(self, *args):
        raise NotImplementedError
