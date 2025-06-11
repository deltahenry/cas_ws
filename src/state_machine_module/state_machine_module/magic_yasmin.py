import time
import threading

import rclpy
from rclpy.node import Node

from common_msgs.msg import ButtonCommand, MotionState

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

#parameters
timer_period = 1.0  # seconds

#button state
stop_button = False
init_button = False
reselect_button = False
picker_button = False
assembly_button = False
error_handler_button = False

#motion state
motion_finished = False
init_finished = False
picker_finished = False
assembly_finished = False
manual_aligment_finished = False
auto_aligment_finished = False
system_error = False


# Define the StartState class, inheriting from the State class(Automatically run)
class StartState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2", "outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state START")

        while True:
            global init_button

            if stop_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to STOP state")
                return "outcome1"
            elif system_error:
                yasmin.YASMIN_LOG_INFO("System error detected, proceeding to WARNING state")
                return "outcome2"
            elif init_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to INIT state")
                
                print("button_cmd rewrite in FSM and UI")
                init_button = False  # Reset the init button state
                rewrite_button_cmd()  # Publish the button command
                
                return "outcome3"
            else:
                yasmin.YASMIN_LOG_INFO("Waiting for Init button press...")
            time.sleep(timer_period)

# Define the InitState class, inheriting from the State class
class InitState(State):

    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2", "outcome3"])
        print("publishing motion_cmd['init']")
    
    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state INIT")
        while True:
            if stop_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to STOP state")
                return "outcome1"
            elif system_error:
                yasmin.YASMIN_LOG_INFO("System error detected, proceeding to WARNING state")
                return "outcome2"
            elif init_finished:
                yasmin.YASMIN_LOG_INFO("Init finished, proceeding to IDLE state")               
                return "outcome3"
            else:
                yasmin.YASMIN_LOG_INFO("Waiting for Init finish...")
            time.sleep(timer_period)

# Define the IdleState class, inheriting from the State class
class IdleState(State):

    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1", "outcome2", "outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state IDLE")
        while True:
            if stop_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to STOP state")
                return "outcome1"
            elif system_error:
                yasmin.YASMIN_LOG_INFO("System error detected, proceeding to WARNING state")
                return "outcome2"
            elif picker_button:
                yasmin.YASMIN_LOG_INFO("Picker button pressed, proceeding to Manual Aligment state")
                return "outcome3"
            elif assembly_button:
                yasmin.YASMIN_LOG_INFO("Assembly button pressed, proceeding to Manual Aligment state")   
                return "outcome3"             
            else:            
                yasmin.YASMIN_LOG_INFO("Waiting for Picker or Assembly button press...")
            time.sleep(timer_period)

# Define the ManualAligmentState class, inheriting from the State class
class ManualAligmentState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1", "outcome2", "outcome3", "outcome4"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state MANUAL_ALIGMENT")
        while True:

            global reselect_button, picker_button, assembly_button, manual_aligment_finished

            if stop_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to STOP state")
                return "outcome1"
            elif system_error:
                yasmin.YASMIN_LOG_INFO("System error detected, proceeding to WARNING state")
                return "outcome2"
            elif reselect_button:
                yasmin.YASMIN_LOG_INFO("Reselect button pressed, proceeding to IDLE state")
                
                print("button_cmd rewrite in FSM and UI]")
                reselect_button = False
                picker_button = False
                assembly_button = False 
                rewrite_button_cmd()

                return "outcome3"
            elif manual_aligment_finished:
                yasmin.YASMIN_LOG_INFO("Manual Aligment finished, proceeding to Auto Aligment state")
                
                print("button_cmd rewrite in FSM and UI")
                manual_aligment_finished = False
                rewrite_motion_state()
                
                return "outcome4"
            else:
                yasmin.YASMIN_LOG_INFO("Waiting for Maunal Aligment finish...")
            time.sleep(timer_period)

# Define the AutoAligmentState class, inheriting from the State class
class AutoAligmentState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1", "outcome2", "outcome3", "outcome4"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state AUTO_ALIGMENT")
        while True:
            global picker_button, assembly_button, auto_aligment_finished

            if stop_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to STOP state")
                return "outcome1"
            elif system_error:
                yasmin.YASMIN_LOG_INFO("System error detected, proceeding to WARNING state")
                return "outcome2"
            elif picker_button and auto_aligment_finished:
                yasmin.YASMIN_LOG_INFO("Auto Aligment finished, proceeding to PICKER state")
                
                print("button_cmd rewrite in FSM and UI")
                picker_button = False
                rewrite_button_cmd()
                
                print("motion_state rewrite in FSM and UI")
                auto_aligment_finished = False
                rewrite_motion_state()
                
                return "outcome3"
            elif assembly_button and auto_aligment_finished:
                yasmin.YASMIN_LOG_INFO("Auto Aligment finished, proceeding to ASSEMBLY state")

                print("button_cmd rewrite in FSM and UI")
                assembly_button = False
                rewrite_button_cmd()
                
                print("motion_state rewrite in FSM and UI")
                auto_aligment_finished = False
                rewrite_motion_state()

                return "outcome4"
            else:
                yasmin.YASMIN_LOG_INFO("Waiting for Auto Aligment finish...")
            time.sleep(timer_period)

# Define the PickerState class, inheriting from the State class
class PickerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1", "outcome2", "outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state PICKER")
        while True:
            global picker_finished

            if stop_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to STOP state")
                return "outcome1"
            elif system_error:
                yasmin.YASMIN_LOG_INFO("System error detected, proceeding to WARNING state")
                return "outcome2"
            elif picker_finished:
                yasmin.YASMIN_LOG_INFO("Picker finished, proceeding to IDLE state")

                print("motion_state rewrite in FSM and UI")
                picker_finished = False
                rewrite_motion_state()

                return "outcome3"
            else:
                yasmin.YASMIN_LOG_INFO("Waiting for Pick finish...")
            time.sleep(timer_period)

# Define the AssemblyState class, inheriting from the State class
class AssemblyState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1", "outcome2", "outcome3"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state ASSEMBLY")
        while True:
            global assembly_finished

            if stop_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to STOP state")
                return "outcome1"
            elif system_error:
                yasmin.YASMIN_LOG_INFO("System error detected, proceeding to WARNING state")
                return "outcome2"
            elif assembly_finished:
                yasmin.YASMIN_LOG_INFO("Assembly finished, proceeding to IDLE state")

                print("motion_state rewrite in FSM and UI")
                assembly_finished = False
                rewrite_motion_state()
                
                return "outcome3"
            else:
                yasmin.YASMIN_LOG_INFO("Waiting for Assembly finish...")
            time.sleep(timer_period)

# Define the WarningState class, inheriting from the State class
class WarningState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1", "outcome2"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state WARNING")
        while True:
            if stop_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to STOP state")
                return "outcome1"
            elif error_handler_button:
                yasmin.YASMIN_LOG_INFO("Error handler button pressed, proceeding to ERROR_HANDLER state")
                return "outcome2"
            else:
                yasmin.YASMIN_LOG_INFO("Waiting for Error Handler button press...")
            time.sleep(timer_period)

# Define the ErrorHandlerState class, inheriting from the State class
class ErrorHandlerState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome1", "outcome2"])

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state ERROR_HANDLER")
        while True:
            if stop_button:
                yasmin.YASMIN_LOG_INFO("Button pressed, proceeding to STOP state")
                return "outcome1"
            elif init_button:
                yasmin.YASMIN_LOG_INFO("Init button pressed, proceeding to IDLE state")
                return "outcome2"
            else:
                yasmin.YASMIN_LOG_INFO("Waiting for Init button press...")
            time.sleep(timer_period)

class MagicCube_FSM:
    def __init__(self):
        yasmin.YASMIN_LOG_INFO("yasmin_demo")
        
        #open the blackboard for shareing data in the FSM
        self.blackboard = Blackboard()
        self.blackboard["test_str"] = 1

        # Create a finite state machine (FSM)
        sm = StateMachine(outcomes=["STOP"])
        sm._call_transition_cbs(blackboard=self.blackboard,from_state=None, to_state="START",outcome=None)

        # Add states to the FSM
        sm.add_state(
            "START",
            StartState(),
            transitions={
                "outcome1": "STOP",
                "outcome2": "WARNING",
                "outcome3": "INIT",
            },
        )
        sm.add_state(
            "INIT",
            InitState(),
            transitions={
                "outcome1": "STOP",
                "outcome2": "WARNING",
                "outcome3": "IDLE",
            },
        )
        sm.add_state(
            "IDLE",
            IdleState(),
            transitions={
                "outcome1": "STOP",
                "outcome2": "WARNING",
                "outcome3": "MANUAL_ALIGMENT",
            },
        )
        sm.add_state(
            "MANUAL_ALIGMENT",
            ManualAligmentState(),
            transitions={
                "outcome1": "STOP",
                "outcome2": "WARNING",
                "outcome3": "IDLE",
                "outcome4": "AUTO_ALIGMENT",
            },
        )
        sm.add_state(
            "AUTO_ALIGMENT",
            AutoAligmentState(),
            transitions={
                "outcome1": "STOP",
                "outcome2": "WARNING",
                "outcome3": "PICKER",
                "outcome4": "ASSEMBLY",
            },
        )
        sm.add_state(
            "PICKER",
            PickerState(),  
            transitions={
                "outcome1": "STOP",
                "outcome2": "WARNING",
                "outcome3": "IDLE",
            },
        )
        sm.add_state(
            "ASSEMBLY",
            AssemblyState(),
            transitions={
                "outcome1": "STOP",
                "outcome2": "WARNING",
                "outcome3": "IDLE",
            },
        )
        sm.add_state(
            "WARNING",
            WarningState(),
            transitions={
                "outcome1": "STOP",
                "outcome2": "ERROR_HANDLER",
            },
        )
        sm.add_state(
            "ERROR_HANDLER",
            ErrorHandlerState(),
            transitions={
                "outcome1": "STOP",
                "outcome2": "IDLE",
            },
        )


        # Publish FSM information for visualization
        YasminViewerPub("yasmin_demo", sm)

        # Execute the FSM
        try:
            outcome = sm(blackboard=self.blackboard)
            yasmin.YASMIN_LOG_INFO(outcome)
        except KeyboardInterrupt:
            if sm.is_running():
                sm.cancel_state()

def rewrite_button_cmd():
    rewrite_button_node = MyROSNode()
    rewrite_button_node.button_cmd_publisher.publish(
        ButtonCommand(
            stop_button=stop_button,
            init_button=init_button,
            reselect_button=reselect_button,
            picker_button=picker_button,
            assembly_button=assembly_button,
            error_handler_button=error_handler_button
        )
    )

def rewrite_motion_state():
    rewrite_motion_node  = MyROSNode()
    rewrite_motion_node.motion_state_publisher.publish(
        MotionState(
            motion_finished=motion_finished,
            init_finished=init_finished,
            picker_finished=picker_finished,
            assembly_finished=assembly_finished,
            manual_aligment_finished=manual_aligment_finished,
            auto_aligment_finished=auto_aligment_finished,
            system_error=system_error
        )
    )


class MyROSNode(Node):
    def __init__(self):
        super().__init__('my_threaded_node')

        self.button_cmd_publisher = self.create_publisher(ButtonCommand, "/button_cmd", 10)
        self.motion_state_publisher = self.create_publisher(MotionState, "/motion_state", 10)

        self.button_cmd_subscriber = self.create_subscription(
            ButtonCommand,
            "/button_cmd",
            self.button_cmd_callback,
            10
        )
        self.motion_state_subscriber = self.create_subscription(
            MotionState,
            "/motion_state",
            self.motion_state_callback,
            10
        )
    def button_cmd_callback(self, msg=ButtonCommand):
        global stop_button, init_button, reselect_button, picker_button, assembly_button, error_handler_button
        stop_button = msg.stop_button
        init_button = msg.init_button
        reselect_button = msg.reselect_button
        picker_button = msg.picker_button
        assembly_button = msg.assembly_button
        error_handler_button = msg.error_handler_button
        self.get_logger().info(f"Received ButtonCommand: {msg}")
    
    def motion_state_callback(self, msg=MotionState):
        global motion_finished, init_finished, picker_finished, assembly_finished, manual_aligment_finished, auto_aligment_finished, system_error
        motion_finished = msg.motion_finished
        init_finished = msg.init_finished
        picker_finished = msg.picker_finished
        assembly_finished = msg.assembly_finished
        manual_aligment_finished = msg.manual_aligment_finished
        auto_aligment_finished = msg.auto_aligment_finished
        system_error = msg.system_error
        self.get_logger().info(f"Received MotionState: {msg}")
    
    

def ros2_thread():
    rclpy.init()
    node = MyROSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

def fsm_thread():
    MagicCube_FSM()

# Main function to initialize and run the state machine
def main():
    t1 = threading.Thread(target=ros2_thread)
    t2 = threading.Thread(target=fsm_thread)

    t1.start()
    t2.start()

    t1.join()
    t2.join()



if __name__ == "__main__":
    main()
