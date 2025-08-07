# ui_app/ui_components/ui_dido.py
from common_msgs.msg import DIDOCmd  # Custom message: string name, bool state

class DIDOController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        # Initialize internal DI/DO state
        self.ros_node.dido_cmd = {}

        # === Setup DO Buttons (48) ===
        for i in range(1, 49):
            do_name = f"DO{i}"
            self.ros_node.dido_cmd[do_name] = False
            button = getattr(self.ui, do_name)
            button.toggled.connect(
                lambda checked, name=do_name: self.toggle_io(name, checked)
            )

        # === Setup DI Buttons (16) ===
        for i in range(1, 17):
            di_name = f"DI{i}"
            self.ros_node.dido_cmd[di_name] = False
            button = getattr(self.ui, di_name)
            button.toggled.connect(
                lambda checked, name=di_name: self.toggle_io(name, checked)
            )

    def toggle_io(self, name, checked):
        """Toggle DI/DO button, update internal state, and publish ROS message."""
        self.ros_node.dido_cmd[name] = checked

        msg = DIDOCmd()
        msg.name = name
        msg.state = checked 
        self.ros_node.dido_control_publisher.publish(msg)

        print(f"[UI] {name} state: {checked}")
