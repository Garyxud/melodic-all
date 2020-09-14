#!/usr/bin/env python
import os
import yaml
import rospy
import datetime
import threading
from sensor_msgs.msg import Joy
from controller_manager_msgs.srv import ListControllers

# Imports from custom files of this package
from helper import *
from controllers_info import *

try: 
    import msvcrt
except ImportError:
    import sys, termios

ui = [None] # mutable hack. Thanks python!

# Translate Twist message fields to selection text
def twistToText():
    m = "Choose an axis to move with your current joy binding:\n"
    m += "0. linear.x\t1. linear.y\t2. linear.z\n"
    m += "3. angular.x\t4. angular.y\t5. angular.z\n"
    return m

# Translate JointTrajectory message fields to selection text
def jointTrajToText():
    m = "Choose a category to move with your current joy binding:\n"
    m += "0. position\t1. velocity\n"
    m += "2. acceleration\t3. effort\n"
    return m

# Translate a joyAction object to a dict for yaml
def joyActionsToDict(actions):
    d = dict()
    for i, action in enumerate(actions):
        d[i] = {"button": action.button,
                "axis": action.axis,
                "value": action.value,
                "joint": action.joint,
                "msg_field": action.msg_field
                }
    return d

# Translate the list of controllers to a dict for yaml
def controllersToDict():
    d = dict()
    for controller in controllers:
        if controller.joyActions:
            d[controller.name] = {"type": controller.type,
                                    "topic": controller.topic,
                                    "actions": joyActionsToDict(controller.joyActions)
                                }
    return d

# Save the list of controllers to a .yaml file
def save():
    print("Saving...")
    filename = config_file_location+ os.sep+"config_"+datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")+".yaml"
    with open(filename, "w") as outfile:
        yaml.dump(controllersToDict(), outfile, default_flow_style=False)
    print("Success!")
    print("Saved at {}".format(filename))

# Python agnostic user input with accepted answers support
def youTalkinToMe(prompt, accepted_ans):
    while(True):
        flush()
        print(prompt)
        if accepted_ans:
            print("(Accepted answers: {})".format(accepted_ans))
        inp = ""
        try:
            # Python 2
            inp = raw_input()
        except:
            # Python 3
            inp = input()
        if not accepted_ans or inp in accepted_ans:
            return inp

# Python agnostic user input for floats
def youTalkinToMeAboutFloats(prompt):
    while(True):
        flush()
        print(prompt)
        inp = ""
        try:
            # Python 2
            inp = raw_input()
        except:
            # Python 3
            inp = input()
        try:
            return float(inp)
        except:
            print("Please provide a number!")

# Python agnostic user input to be run in a thread
def youTalkinToMeAboutThreads(prompt):
    global ui
    print(prompt)
    while(read_joy):
        flush()
        if saved_buti != -1 or saved_axi != -1:
            try:
                # Python 2
                ui[0] = raw_input()
            except:
                # Python 3
                ui[0] = input()

# Translate user joystick input to human readable text
# Also, offer the expected behaviour when unpressing buttons/axes
# or when pressing new buttons/axes that need to delete old ones
def joyInLife(msg):
    global saved_axi, saved_buti
    max_ax = max(map(abs, msg.axes))
    axis_i = -1
    button_i = -1
    if min(msg.axes) == -max_ax and max_ax !=0:
        axis_i = -1 if max_ax == 0 else msg.axes.index(-max_ax)
    elif max_ax != 0:
        axis_i = -1 if max_ax == 0 else msg.axes.index(max_ax)
    if 1 in msg.buttons:
        button_i = msg.buttons.index(1)
    combo = None
    if button_i != -1 and button_i != saved_buti:
        saved_axi = -1
    if axis_i != -1 and axis_i != saved_axi:
        saved_buti = -1
    saved_buti = button_i if button_i != -1 else saved_buti
    saved_axi = axis_i if axis_i != -1 else saved_axi
    if saved_axi >= 0:
        combo = "[Axis{}]".format(saved_axi)
    if saved_buti >= 0:
        if combo is not None:
            combo += "+[Button{}]".format(saved_buti)
        else:
            combo = "[Button{}]".format(saved_buti)
    return combo

# The joystic callback
def joyCallback(msg):
    if read_joy:
        clear()
        print("Controls for {}".format(curr_controller.name))
        print(joyInLife(msg))
        if saved_axi != -1 or saved_buti != -1:
            print("Press enter on your keyboard to continue with the current configuration...")

# OS agnostic terminal clearance
def clear():
    os.system("cls" if os.name == "nt" else "clear")

# OS agnostic terminal flushing
def flush():
    try:
        while msvcrt.kbhit():
            msvcrt.getch()
    except:
        termios.tcflush(sys.stdin, termios.TCIOFLUSH)

# The main setup procedure
def configureJoyActions():
    global read_joy, keepItUp, curr_controller, saved_buti, saved_axi, ui
    ans = youTalkinToMe("Do you want to see all the discovered controllers?", YES+NO+QUIT)
    if ans in YES:
        print(controllers)
        print("---")
    elif ans in QUIT:
        keepItUp = False
        return
    print("Initiating joystick configuration...")
    youTalkinToMe("Press any enter to continue...", [])
    clear()
    for controller in controllers:
        cnt = 0
        if controller.type in supported_controllers:
            while(True):
                ans = ""
                if cnt == 0:
                    ans = youTalkinToMe("Do you want to assign a joystick command to {}?".format(controller.name), YES+NO+QUIT)
                else:
                    ans = youTalkinToMe("Do you want to assign another joystick command to {}?".format(controller.name), YES+NO+QUIT)
                if ans in QUIT:
                    keepItUp = False
                    return
                elif ans in YES:
                    curr_controller = controller
                    for joint in controller.joints:
                        msg_field = ""
                        if controller.type in ignore_controller_joints:
                            ans = "y"
                        else:
                            ans = youTalkinToMe("[{}] Do you want to assign a joystick command to {}?".format(controller.name, joint), YES+NO+QUIT)
                        if ans in YES:
                            cnt += 1
                            read_joy = True
                            print("Now use your controller to set a new configuration...")
                            t = threading.Thread(target=youTalkinToMeAboutThreads, args=("Don't touch your keyboard now!",))
                            t.deamon = True
                            t.start()
                            while read_joy and keepItUp and not rospy.is_shutdown():
                                if ui[0] is not None and (saved_axi != -1 or saved_buti != -1):
                                    read_joy = False

                            if controller.type in twist_controllers:
                                ans = youTalkinToMe(twistToText(), list(twisty_dict.keys()))
                                msg_field = twisty_dict[ans]
                            elif controller.type in joint_traj_controllers:
                                ans = youTalkinToMe(jointTrajToText(), list(jt_dict.keys()))
                                msg_field = jt_dict[ans]

                            val = -1
                            if controller.type not in non_step_controllers:
                                val = youTalkinToMeAboutFloats("Please provide a joint step value:")
                            else:
                                val = youTalkinToMeAboutFloats("Please provide a cmd_vel value:")

                            ans = youTalkinToMe("Do you want to save the current joy binding?", YES+NO+QUIT)
                            if ans in YES:
                                controller.joyActions.append(JoyAction(saved_buti, saved_axi, val, joint, msg_field))
                                print("Saved! (Not on disk yet!)")
                                print("---")
                            elif ans in QUIT:
                                keepItUp = False
                                return
                            ui = [None]
                            saved_buti = -1
                            saved_axi = -1
                            if controller.type in ignore_controller_joints:
                                break
                        elif ans in QUIT:
                            keepItUp = False
                            return
                elif ans in NO:
                    break

    while(keepItUp):
        ans = youTalkinToMe("Do you want to save your configuration?", YES+NO+QUIT)
        if ans in YES:
            save()
            keepItUp = False
        elif ans in QUIT:
            keepItUp = False
        elif ans in NO:
            ans = youTalkinToMe("Are you sure?", YES+NO+QUIT)
            if ans in YES or ans in QUIT:
                keepItUp = False

# Initialisations and other boring stuff
def main():
    global config_file_location
    rospy.init_node("jointstick_setup")
    config_file_location = rospy.get_param("config_file_location", os.path.expanduser("~"))
    print("Config file save location was set as: {}".format(config_file_location))
    rospy.Subscriber("/joy", Joy, joyCallback)
    controllers_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)

    print("Reading available controllers....")

    rospy.wait_for_service("controller_manager/list_controllers")

    c = controllers_srv().controller

    for controller in c:
        if controller.type != "joint_state_controller/JointStateController":
            controllers.append(Controller(controller.name, controller.type, [], controller.claimed_resources[0].resources, controller.name+topic_extension[controller.type]))

    print("Done!")

    configureJoyActions()

    while not rospy.is_shutdown() and keepItUp:
        rospy.spin()
    print("Byeee!")

main()
