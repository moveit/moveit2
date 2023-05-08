import subprocess


class GripperClient:
    def __init__(self, action_name):
        self.action_name = action_name

    def send_gripper_command(self, position, max_effort):
        subprocess.run(
            [
                "ros2",
                "action",
                "send_goal",
                self.action_name,
                "control_msgs/action/GripperCommand",
                "{command: { position: "
                + str(position)
                + ", max_effort: "
                + str(max_effort)
                + "}}",
            ]
        )
