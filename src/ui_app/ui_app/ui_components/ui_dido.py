# ui_app/ui_components/ui_dido.py
from common_msgs.msg import DIDOCmd  # Custom message: string name, bool state

class DIDOController:
    def __init__(self, ui, ros_node):
        self.ui = ui
        self.ros_node = ros_node

        # Initialize internal DI/DO state
        self._do_buttons = {}
        self._di_buttons = {}

        self.ros_node.dido_cmd = {}

        # --- DO buttons (UI -> ROS) ---
        for i in range(1, 49):
            name = f"DO{i}"
            btn = getattr(self.ui, name, None)
            if btn is None:
                continue
            self._do_buttons[name] = btn
            self.ros_node.dido_cmd[name] = False
            # publish on user toggle
            btn.clicked.connect(lambda checked, n=name: self.toggle_do(n, checked))

        # === Setup DI Buttons (16) ===
        for i in range(1, 17):
            name = f"DI{i}"
            btn = getattr(self.ui, name, None)
            if btn is None:
                continue
            self._di_buttons[name] = btn
            self.ros_node.dido_cmd[name] = False

    # def toggle_io(self, name, checked):
    #     """Toggle DI/DO button, update internal state, and publish ROS message."""
    #     self.ros_node.dido_cmd[name] = checked

    #     msg = DIDOCmd()
    #     msg.name = name
    #     msg.state = checked 
    #     self.ros_node.dido_control_publisher.publish(msg)

    #     print(f"[UI] {name} state: {checked}")

    # ===== DO path: publish commands =====
    def toggle_do(self, name: str, checked: bool):
    
        self.ros_node.dido_cmd[name] = checked
        msg = DIDOCmd()
        msg.name = name
        msg.state = checked
        self.ros_node.dido_control_publisher.publish(msg)
        print(f"[UI->ROS] {name} -> {checked}")


    # def toggle_do(self, name: str, checked: bool):
    #     btn = self._do_buttons.get(name)
    #     if not btn:
    #         return

    #     # Revert visual state immediately (stay at previous) and disable until confirm
    #     # was_blocked = btn.blockSignals(True)
    #     btn.setChecked(not checked)
    #     # btn.blockSignals(was_blocked)
    #     # btn.setEnabled(False)

    #     # Publish the desired state to ROS
    #     msg = DIDOCmd()
    #     msg.name = name
    #     msg.state = checked
    #     self.ros_node.dido_control_publisher.publish(msg)
    #     print(f"[UI->ROS] Request: {name} -> {checked}")

    def update_do(self, name: str, state: bool):
        btn = self._do_buttons.get(name)
        if not btn:
            return
        # was_blocked = btn.blockSignals(True)
        btn.setChecked(state)      # now reflect the real state
        # btn.blockSignals(was_blocked)
        btn.setEnabled(True)
        self.ros_node.dido_cmd[name] = state

    # ===== DI path: reflect hardware state (no publish) =====
    def update_di(self, name: str, state: bool):
        btn = self._di_buttons.get(name)
        if not btn:
            return
        was_blocked = btn.blockSignals(True)   # make sure no accidental signals
        btn.setChecked(state)
        btn.blockSignals(was_blocked)
        # optional styling
        # btn.setStyleSheet("background-color: #0B76A0; color: white;" if state else "")
        self.ros_node.dido_cmd[name] = state