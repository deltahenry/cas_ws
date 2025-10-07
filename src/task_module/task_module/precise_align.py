import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32,Int32MultiArray
from common_msgs.msg import StateCmd,TaskCmd,MotionCmd,TaskState,MotionState,ForkCmd,ForkState,Recipe

#parameters
timer_period = 0.5  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        self.state_cmd ={
            'pause_button': False,
            'stop_button': False,
        }

        self.task_cmd = "idle"  # rough align,precise align,pick,assembly
      
        self.current_height = 0.0
        self.forkstate = "idle"

        self.target_mode = "idle" # idle,pick,assembly  
        self.target_height = 150.0

        self.compensate_state = "idle" # idle,done,fail
       
        # 初始化 ROS2 Node
        #subscriber
        super().__init__('data_node')
        self.state_cmd_subscriber = self.create_subscription(
            StateCmd,
            '/state_cmd',
            self.state_cmd_callback,
            10
        )

        self.task_cmd_subscriber = self.create_subscription(
            TaskCmd,
            '/task_cmd',
            self.task_cmd_callback,
            10
        )
        
        self.depth_data_subscriber = self.create_subscription(
            Float32MultiArray,
            "/depth_data",
            self.depth_data_callback,
            10
        )

        self.height_info_subscriber = self.create_subscription(
            Int32,
            'lr_distance',
            self.height_info_callback,
            10
        )

        self.fork_state_subscriber = self.create_subscription(
            ForkState,
            'fork_state',
            self.fork_state_callback,
            10
        )

        self.recipe_data_subscriber = self.create_subscription(
            Recipe,
            'recipe_data',
            self.recipe_callback,
            10
        )

        self.compensate_state_subscriber = self.create_subscription(
            TaskState,
            '/task_state_compensate',
            self.compensate_state_callback,
            10
        )

        #publisher
        self.precise_align_state_publisher = self.create_publisher(TaskState, '/task_state_precise_align', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.detection_cmd_publisher = self.create_publisher(String,'/detection_task',10)
        self.fork_cmd_publisher = self.create_publisher(ForkCmd, 'fork_cmd', 10)
        self.laser_cmd_publisher = self.create_publisher(Int32MultiArray,'/laser_io_cmd',10)
        self.compensate_cmd_publisher = self.create_publisher(TaskCmd, '/compensate_cmd', 10)
        
    def publish_fork_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.fork_cmd_publisher.publish(msg)
        self.get_logger().info(f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}")

    def state_cmd_callback(self, msg: StateCmd):
        print(f"接收到狀態命令: {msg}")
        # 在這裡可以處理狀態命令
        self.state_cmd = {
            'pause_button': msg.pause_button,
            'stop_button': msg.stop_button,
        }

    def task_cmd_callback(self, msg: TaskCmd):
        print(f"接收到任務命令: {msg.mode}")
        # 在這裡可以處理任務命令
        self.task_cmd = msg.mode
        
    def depth_data_callback(self, msg: Float32MultiArray):
        print(f"接收到深度數據: {msg.data}")
        # 在這裡可以處理深度數據
        self.depth_data = msg.data      
        # 更新深度數據
        if len(self.depth_data) >= 2:
            self.depth_data[0] = msg.data[0]
            self.depth_data[1] = msg.data[1]        
        else:
            self.get_logger().warn("接收到的深度數據長度不足，無法更新。")

    def height_info_callback(self,msg: Int32):
        """接收來自LR Sensor的高度信息"""
        self.get_logger().info(f"Received height info: {msg.data} mm")
        self.current_height = msg.data

    def fork_state_callback(self, msg: ForkState):
        self.forkstate = msg.state  # 假設 ForkState 有個 .state 屬性

    def recipe_callback(self, msg: Recipe):
        self.target_mode = msg.mode
        self.target_height = msg.height

    def compensate_state_callback(self, msg: TaskState):
        """接收補償任務狀態"""
        print(f"接收到補償任務狀態: {msg.state}")
        self.compensate_state = msg.state

class PreciseAlignState(Enum):
    IDLE = "idle"
    INIT = "init"
    MOVE_DETECT = "move_to_detect"
    MOVE_ACT = "move_to_act"
    ALIGN = "align"
    DONE = "done"
    FAIL = "fail"

class PreciseAlignFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = PreciseAlignState.IDLE  # 初始狀態
        self.data_node = data_node
        self.send_fork_cmd = False
        self.act_height = 0.0 

        states = [
            PreciseAlignState.IDLE.value,
            PreciseAlignState.INIT.value,
            PreciseAlignState.MOVE_DETECT.value,
            PreciseAlignState.ALIGN.value,
            PreciseAlignState.MOVE_ACT.value,
            PreciseAlignState.DONE.value,
            PreciseAlignState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_init', 'source': PreciseAlignState.IDLE.value, 'dest': PreciseAlignState.INIT.value},
            {'trigger': 'init_to_move_detect', 'source': PreciseAlignState.INIT.value, 'dest': PreciseAlignState.MOVE_DETECT.value},
            {'trigger': 'move_detect_to_align', 'source': PreciseAlignState.MOVE_DETECT.value, 'dest': PreciseAlignState.ALIGN.value},
            # {'trigger': 'align_to_done', 'source': PreciseAlignState.ALIGN.value, 'dest': PreciseAlignState.DONE.value},
            {'trigger': 'align_to_move_act', 'source': PreciseAlignState.ALIGN.value, 'dest': PreciseAlignState.MOVE_ACT.value},
            {'trigger': 'move_act_to_done', 'source': PreciseAlignState.MOVE_ACT.value, 'dest': PreciseAlignState.DONE.value},
            {'trigger': 'return_to_idle', 'source': '*', 'dest': PreciseAlignState.IDLE.value},
            {'trigger': 'to_fail', 'source': '*', 'dest': PreciseAlignState.FAIL.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = PreciseAlignState(self.state)


    def reset_parameters(self):
        """重置參數"""
        self.data_node.task_cmd = "idle"

        self.data_node.target_mode = "idle" #idle,pick,assembly

        self.send_fork_cmd = False  # 用於控制叉車命令的發送

        self.data_node.compensate_state = "idle" # idle,done,fail

        self.data_node.state_cmd = {
            'pause_button': False,
        }
        self.send_fork_cmd = False

        self.act_height = 0.0
        
    def step(self):
        if self.data_node.state_cmd.get("pause_button", False):
            print("[PreciseAlignmentFSM] 被暫停中")
            return  # 暫停狀態，不進行任何操作

        elif self.data_node.state_cmd.get("stop_button", False):
            print("[RoughAlignmentFSM] 被停止，返回空閒狀態")
            self.reset_parameters()
            self.return_to_idle()  # 返回到空閒狀態
            return
        
        elif self.data_node.task_cmd == "precise_align":
            print("[PreciseAlignmentFSM] 開始手動對齊任務")
            self.run()

        else:
            print("[PreciseAlignmentFSM] 手動對齊任務未啟動，等待中")
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

        # 任務完成或失敗時自動清除任務旗標

    def run(self):

        if self.state == PreciseAlignState.IDLE.value:
            print("[PreciseAlignmentFSM] 等待開始")
            if self.data_node.task_cmd == "precise_align":
                print("[PreciseAlignmentFSM] 開始對齊")
                self.idle_to_init()
            else:
                print("[PreciseAlignmentFSM] 對齊任務未啟動，等待中")
        
        elif self.state == PreciseAlignState.INIT.value:   
            self.laser_cmd("laser_open")  # 開啟雷射
            print("[PreciseAlignmentFSM] 初始化階段")

            if self.data_node.target_mode in ["pick","assembly"]:
                print(f"[PreciseAlignmentFSM] 選擇運行模式: {self.data_node.target_mode}，進入移動到檢測位置階段")
                self.init_to_move_detect()
            else:
                print("[PreciseAlignmentFSM] 未選擇運行模式，等待人為選擇")
                return

        elif self.state == PreciseAlignState.MOVE_DETECT.value:
            regular_cabinent_height = 120.0
            special_cabinent_height = 113.0
            mecheye_height = 10.0
            tolerance = 1.0

            if abs(self.data_node.target_height) - 226.0 < 10.0: #2nd cabinent
                height_cmd = self.data_node.target_height - special_cabinent_height - mecheye_height
            else:
                height_cmd = self.data_node.target_height- regular_cabinent_height - mecheye_height

            # height_cmd = self.data_node.target_height

            if not self.send_fork_cmd:
                self.fork_cmd(mode="run", speed="slow", direction="down", distance=height_cmd)
                self.send_fork_cmd = True
            
            else:
                if abs(self.data_node.current_height - height_cmd) <= tolerance and self.data_node.forkstate == "idle":
                    self.send_fork_cmd = False
                    print("[PreciseAlignmentFSM] 叉車已到達目標高度，進入電池檢測階段")
                    self.move_detect_to_align()
                    #open rough align
                    self.data_node.compensate_cmd_publisher.publish(TaskCmd(mode='l_shape'))
                else:
                    print("waiting")
                            
        elif self.state == PreciseAlignState.ALIGN.value:

            if self.data_node.compensate_state == "done":
                print("粗對齊完成，進入深度檢查階段")
                # self.align_to_done()
                self.align_to_move_act()

                #close rough align
                self.data_node.compensate_cmd_publisher.publish(TaskCmd(mode='stop'))

            elif self.data_node.compensate_state == "fail":
                print("粗對齊失敗，進入失敗狀態")
                #close rough align
                self.data_node.compensate_cmd_publisher.publish(TaskCmd(mode='stop'))
                self.to_fail()

            else:
                print("waiting for precise align done")
        
        elif self.state == PreciseAlignState.MOVE_ACT.value:
            # height_cmd = self.data_node.target_height
            tolerance = 1.0

            if not self.send_fork_cmd:
                if abs(self.data_node.target_height) - 226.0 < 10.0: #2nd cabinent
                    self.act_height = self.data_node.current_height + 123.0
                else:
                    self.act_height = self.data_node.current_height + 130.0
                    
                self.fork_cmd(mode="run", speed="slow", direction="down", distance=self.act_height)
                self.send_fork_cmd = True
                
            else:
                if abs(self.data_node.current_height - self.act_height) <= tolerance and self.data_node.forkstate == "idle":
                    self.send_fork_cmd = False
                    print("[PreciseAlignmentFSM] 叉車已到達目標高度，進入電池檢測階段")
                    self.move_act_to_done()
                else:
                    print("waiting")
            
        elif self.state == PreciseAlignState.DONE.value:
            self.laser_cmd("laser_close")  # 開啟雷射
            print("[PreciseAlignmentFSM] 對齊任務完成")
            
        elif self.state == PreciseAlignState.FAIL.value:
            print("[PreciseAlignmentFSM] 對齊任務失敗")
            #close_guide_laser
            self.laser_cmd("laser_close")  # 關閉雷射
            self.reset_parameters()
            self.return_to_idle()  # 返回到空閒狀態

    def fork_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.data_node.fork_cmd_publisher.publish(msg)
        print(f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}")
        # self.get_logger().info(f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}")
                    
    def laser_cmd(self, cmd: str):
        """發送雷射命令"""
        if cmd == "laser_open":
            value = [1,1]
            value = Int32MultiArray(data=value)  # 封裝為 Int32MultiArray
            self.data_node.laser_cmd_publisher.publish(value)
        elif cmd == "laser_close":
            value = [0,0]
            value = Int32MultiArray(data=value)  # 封裝為 Int32MultiArray
            self.data_node.laser_cmd_publisher.publish(value)
        print(f"[RoughAlignmentFSM] 發送雷射命令: {cmd}")


def main():
    rclpy.init()
    data = DataNode()                 # ROS2 subscriber node
    system = PreciseAlignFSM(data)    # FSM 實體

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            print(f"[現在狀態] {system.state}")
            # 更新狀態發布
            data.precise_align_state_publisher.publish(
                TaskState(mode="precise_align", state=system.state)
            )
            time.sleep(timer_period)

    except KeyboardInterrupt:
        pass
    finally:
        data.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

# 🏁 若此檔案直接執行，就進入 main()
if __name__ == "__main__":
    main()