# A file with lists of supported controllers and other
# categorisations on those controllers
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory

# A list of controllers that this script can handle
supported_controllers = ["effort_controllers/JointPositionController",
                        "position_controllers/JointTrajectoryController",
                        "diff_drive_controller/DiffDriveController",
                        "steer_drive_controller/SteerDriveController"]

# A list of controllers that do not apply on specific joints
ignore_controller_joints = ["effort_controllers/JointPositionController",
                            "diff_drive_controller/DiffDriveController",
                            "steer_drive_controller/SteerDriveController"]

# A list of controllers that do not need a step value, but an explicit one (like velocity)
non_step_controllers = ["diff_drive_controller/DiffDriveController",
                        "steer_drive_controller/SteerDriveController"]

# A list of controllers that need a user specified message field (e.g. "linear.x")
message_field_controllers = ["position_controllers/JointTrajectoryController",
                            "diff_drive_controller/DiffDriveController",
                            "steer_drive_controller/SteerDriveController"]

# A list of controllers that their command is based on a Twist message
twist_controllers = ["diff_drive_controller/DiffDriveController",
                    "steer_drive_controller/SteerDriveController"]

# A list of controllers that their commands is based on a JointTrajectory message
joint_traj_controllers = ["position_controllers/JointTrajectoryController"]

# A list of controllers that their commands is based on a Float64 message
float_controllers = ["effort_controllers/JointPositionController"]

topic_extension = {
                    "effort_controllers/JointPositionController": "/command",
                    "position_controllers/JointTrajectoryController": "/command",
                    "diff_drive_controller/DiffDriveController": "/cmd_vel",
                    "steer_drive_controller/SteerDriveController": "/cmd_vel"
                }

controller_type_correspondence = {
                        "effort_controllers/JointPositionController": Float64,
                        "position_controllers/JointTrajectoryController": JointTrajectory,
                        "diff_drive_controller/DiffDriveController": Twist,
                        "steer_drive_controller/SteerDriveController": Twist
                    }


