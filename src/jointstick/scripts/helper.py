# A file with helper variables and classes

terminal_text = ""
controllers = []
curr_controller = None
saved_axi = -1
saved_buti = -1

read_joy = False
keepItUp = True

config_file_location = ""

twisty_dict = {
                "0": "linear.x",
                "1": "linear.y",
                "2": "linear.z",
                "3": "angular.x",
                "4": "angular.y",
                "5": "angular.z"
            }

jt_dict = {
            "0": "position",
            "1": "velocity",
            "2": "acceleration",
            "3": "effort"
        }

# Expected user input
YES = ["YES", "yes", "Y", "y"]
NO = ["NO", "no", "N", "n"]
QUIT = ["QUIT", "quit", "Q", "q"]
SAVE = ["SAVE", "save", "S", "s"]

class JoyAction():
    button = -1
    axis = -1
    value = 0
    joint = ""
    msg_field = ""

    def __init__(self, b, a, v, j, mf):
        self.button = b
        self.axis = a
        self.value = v
        self.joint = j
        self.msg_field = mf

    def __repr__(self):
        return "\nButton: {},\nAxis: {},\nValue: {},\nJoint: {},\nMessage Field: {}\n".format(self.button, self.axis, self.value, self.joint, self.msg_field)

class Controller():
    name = ""
    type = ""
    joints = []
    joyActions = []
    topic = ""

    def __init__(self, n, t, ja, j, tt):
        self.name = n
        self.type = t
        self.joyActions = ja
        self.joints = j
        self.topic = tt

    def __repr__(self):
        return "\nName: {},\nType: {},\nActions: {},\nJoints: {},\nTopic: {}\n".format(self.name, self.type, self.joyActions, self.joints, self.topic)
