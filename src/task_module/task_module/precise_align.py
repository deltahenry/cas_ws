import time
import networkx as nx
import matplotlib.pyplot as plt
from transitions import Machine
from functools import wraps
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32
from common_msgs.msg import StateCmd,TaskCmd,MotionCmd,TaskState,MotionState,ForkCmd,ForkState,Recipe

#parameters
timer_period = 0.5  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):

        self.state_cmd ={
            'pause_button': False,
        }

        self.task_cmd = "idle"  # rough align,precise align,pick,assembly

        self.func_cmd = {
            'pick_button': False,
            'push_button': True,
        }
       
        self.depth_data = [500.0,500.0]
        self.point_dist = 1000.0
        self.current_height = 0.0
        self.cabinet_heights = [150, 300, 350, 400, 500]
        self.screw_TF = None
        self.battery_TF = None
        self.forkstate = "idle"

        self.target_mode = "idle" # idle,pick,assembly  
        self.target_height = 80.0
       
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

        self.motion_state_subscriber = self.create_subscription(
            MotionState,
            "/motion_state",
            self.motion_state_callback,
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

        #publisher
        self.precise_align_state_publisher = self.create_publisher(TaskState, '/task_state_precise_align', 10)
        self.motion_state_publisher = self.create_publisher(MotionState, '/motion_state', 10)
        self.motion_cmd_publisher = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.detection_cmd_publisher = self.create_publisher(String,'/detection_task',10)
        self.fork_cmd_publisher = self.create_publisher(ForkCmd, 'fork_cmd', 10)
        
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
        }

    def task_cmd_callback(self, msg: TaskCmd):
        print(f"接收到任務命令: {msg.mode}")
        # 在這裡可以處理任務命令
        self.task_cmd = msg.mode

    def motion_state_callback(self, msg=MotionState):
        print(f"接收到運動狀態: {msg}")
        # 在這裡可以處理運動狀態
        self.motion_states = {
            'motion_finish': msg.motion_finish,
            'init_finish': msg.init_finish,
            'pull_finish': msg.pull_finish,                 
            'push_finish': msg.push_finish,
            'rough_pos_finish': msg.rough_pos_finish,
            'auto_pos_finish': msg.auto_pos_finish,
            'system_error': msg.system_error
        }

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
        # 在這裡可以添加更多的處理邏輯
        # 例如，根據接收到的 recipe 更新其他狀態或觸發其他操作

class PreciseAlignState(Enum):
    IDLE = "idle"
    INIT = "init"
    SCREW_DETECT = "screw_detect"
    SCREW_ALIGN = "screw_align"
    MOVE_FORKLIFT = "move_forklift"
    BATTERY_DETECT = "battery_detect"
    BATTERY_ALIGN = "battery_align"
    DONE = "done"
    FAIL = "fail"

class PreciseAlignFSM(Machine):
    def __init__(self, data_node: DataNode):
        self.phase = PreciseAlignState.IDLE  # 初始狀態
        self.data_node = data_node
        self.run_mode = "pick"
        self.screw_align_threshold = 10.0
        self.batter_dectect_threshold = 5.0
        self.send_fork_cmd = False

        states = [
            PreciseAlignState.IDLE.value,
            PreciseAlignState.INIT.value,
            PreciseAlignState.SCREW_DETECT.value,
            PreciseAlignState.SCREW_ALIGN.value,
            PreciseAlignState.MOVE_FORKLIFT.value,
            PreciseAlignState.BATTERY_DETECT.value,
            PreciseAlignState.BATTERY_ALIGN.value,
            PreciseAlignState.DONE.value,
            PreciseAlignState.FAIL.value
        ]
        
        transitions = [
            {'trigger': 'idle_to_init', 'source': PreciseAlignState.IDLE.value, 'dest': PreciseAlignState.INIT.value},
            {'trigger': 'init_to_screw_detect', 'source': PreciseAlignState.INIT.value, 'dest': PreciseAlignState.SCREW_DETECT.value},
            {'trigger': 'screw_detect_to_screw_align', 'source': PreciseAlignState.SCREW_DETECT.value, 'dest': PreciseAlignState.SCREW_ALIGN.value},
            {'trigger': 'check_screw_align', 'source': PreciseAlignState.SCREW_ALIGN.value, 'dest': PreciseAlignState.SCREW_DETECT.value},
            {'trigger': 'screw_align_to_move_forklift', 'source': PreciseAlignState.SCREW_ALIGN.value, 'dest': PreciseAlignState.MOVE_FORKLIFT.value},
            {'trigger': 'move_forklift_to_battery_detect', 'source': PreciseAlignState.MOVE_FORKLIFT.value, 'dest': PreciseAlignState.BATTERY_DETECT.value},
            {'trigger': 'battery_detect_to_battery_align', 'source': PreciseAlignState.BATTERY_DETECT.value, 'dest': PreciseAlignState.BATTERY_ALIGN.value},
            {'trigger': 'battery_align_to_done', 'source': PreciseAlignState.BATTERY_ALIGN.value, 'dest': PreciseAlignState.DONE.value},
            {'trigger': 'return_to_idle', 'source': '*', 'dest': PreciseAlignState.IDLE.value},
            {'trigger': 'return_to_fail', 'source': '*', 'dest': PreciseAlignState.FAIL.value},
        ]

        self.machine = Machine(model=self, states=states,transitions=transitions,initial=self.phase.value,
                               auto_transitions=False,after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = PreciseAlignState(self.state)

    def depth_ref(self,run_mode):
        """根據運行模式返回參考深度"""
        if run_mode == "pick":
            return 90.0
        elif run_mode == "push":
            return 90.0

    def reset_parameters(self):
        """重置參數"""
        self.run_mode = "pick"
        self.data_node.depth_data = [600.0, 600.0]
        self.data_node.point_dist = 1000.0
        self.data_node.state_cmd = {
            'pause_button': False,
        }
        self.data_node.func_cmd = {
            'pick_button': False,
            'push_button': True,
        }
        self.send_fork_cmd = False
        
    def step(self):
        if self.data_node.state_cmd.get("pause_button", False):
            print("[PreciseAlignmentFSM] 被暫停中")
        
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
            print("[PreciseAlignmentFSM] 初始化階段")
            if self.data_node.func_cmd.get("pick_button", False):
                self.run_mode = "pick"
                self.init_to_screw_detect()
            elif self.data_node.func_cmd.get("push_button", False):
                self.run_mode = "push"
                self.init_to_screw_detect()
            else:
                print("[PreciseAlignmentFSM] 未選擇運行模式，等待人為選擇")
                return

        elif self.state == PreciseAlignState.SCREW_DETECT.value:
            print("[PreciseAlignmentFSM] 螺絲檢測階段")
            self.screw_detect_to_screw_align()
            # #open detection task screw
            # self.data_node.detection_cmd_publisher.publish(String(data="screw"))
            # #get screw detection result -> TF
            # #NEED 
            # if self.data_node.screw_TF  == None:
            #     print("[PreciseAlignmentFSM] 螺絲檢測未完成，等待中")
            #     return
            # else:
            #     print("[PreciseAlignmentFSM] 螺絲檢測完成，進入螺絲對齊階段")
            #     #close detection task screw
            #     self.data_node.detection_cmd_publisher.publish(String(data="idle"))
            #     # 假設檢測完成後進入下一個狀態
            #     self.screw_detect_to_screw_align()

        elif self.state == PreciseAlignState.SCREW_ALIGN.value:
            print("[PreciseAlignmentFSM] 螺絲對齊階段")
            self.screw_align_to_move_forklift()
            # # 根據螺絲檢測結果進行對齊
            # #NEED
            # screw_TF = self.data_node.screw_TF
            # relative_pose = screw_TF
            
            # pose_cmd  = [self.data_node.current_pose[0]+relative_pose[0],
            #              self.data_node.current_pose[1]+relative_pose[1],
            #              self.data_node.current_pose[2]+relative_pose[2]]
            
            # if norm( relative_pose) > self.screw_align_threshold:
            #     if not self.motor_cmd_sent:
            #         self.motor_cmd_sent(pose_cmd)
            #         self.motor_cmd_sent = True
            #     else:
            #         motor_arrived = self.check_pos(pose_cmd)    
            #         if motor_arrived:
            #             #double check
            #             self.motor_cmd_sent = False
            #             self.data_node.screw_TF = None
            #             self.check_screw_align()
            # else:
            #     #finish screw_align
            #     self.screw_align_to_battery_detect()
        
        elif self.state == PreciseAlignState.MOVE_FORKLIFT.value:
            
            height_cmd = self.data_node.target_height
            tolerance = 5.0

            if not self.send_fork_cmd:
                self.fork_cmd(mode="run", speed="slow", direction="down", distance=height_cmd)
                self.send_fork_cmd = True
            
            else:
                if abs(self.data_node.current_height - height_cmd) <= tolerance and self.data_node.forkstate == "idle":
                    self.send_fork_cmd = False
                    print("[PreciseAlignmentFSM] 叉車已到達目標高度，進入電池檢測階段")
                    self.move_forklift_to_battery_detect()
                else:
                    print("waiting")
            
        elif self.state == PreciseAlignState.BATTERY_DETECT.value:
            print("[PreciseAlignmentFSM] 電池檢測階段")
            self.battery_detect_to_battery_align()
            # #open detection task icp_fit
            # self.data_node.detection_cmd_publisher.publish(String(data="icp_fit"))
            
            # if self.data_node.point_dist > self.batter_dectect_threshold:
            #     #forklift up to top
            #     self.data_node.publish_fork_cmd(mode="run", speed="slow", direction="up", distance="200")
            # else:
            #     #forklift stop
            #     self.data_node.publish_fork_cmd(mode="stop", speed="slow", direction="up", distance="200")
            #     #close detection task icp_fit
            #     self.data_node.detection_cmd_publisher.publish(String(data="idle"))
            #     #go to battery align
            #     self.battery_detect_to_battery_align()
        
        elif self.state == PreciseAlignState.BATTERY_ALIGN.value:
            print("[PreciseAlignmentFSM] 電池對齊階段")
            self.battery_align_to_done()
            # #open detection task l_shape
            # self.data_node.detection_cmd_publisher.publish(String(data="l_shape"))
                
            # current_height = self.data_node.current_height
            # cabinet_heights = self.data_node.cabinet_heights
            
            # #get nearest cabinent
            # height_cmd = min(cabinet_heights, key=lambda h: abs(h - current_height))
            
            # #forklift move 
            # if current_height > height_cmd + 5.0:
            #     #forklift down to height_cmd
            #     self.data_node.publish_fork_cmd(mode="run", speed="slow", direction="down", distance = height_cmd)
            # else:               
            #     #NEED
            #     battery_TF = self.data_node.battery_TF
            #     relative_height = battery_TF[3]
            #     self.data_node.publish_fork_cmd(mode="stop", speed="slow", direction="down", distance = height_cmd)
            
        elif self.state == PreciseAlignState.DONE.value:
            print("[PreciseAlignmentFSM] 對齊任務完成")
            
        elif self.state == PreciseAlignState.FAIL.value:
            print("[PreciseAlignmentFSM] 對齊任務失敗")
            # 在這裡可以添加失敗處理邏輯，例如重試或記錄錯誤

    def fork_cmd(self, mode, speed, direction, distance):
        msg = ForkCmd()
        msg.mode = mode
        msg.speed = speed
        msg.direction = direction
        msg.distance = distance
        self.data_node.fork_cmd_publisher.publish(msg)
        print(f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}")
        # self.get_logger().info(f"Published ForkCmd: mode={mode}, speed={speed}, direction={direction}, distance={distance}")
                
    def sent_motor_cmd(self,pose_cmd):
        """發送馬達初始化命令"""
        msg = MotionCmd()
        msg.command_type = MotionCmd.TYPE_GOTO
        msg.pose_data = [pose_cmd[0], pose_cmd[1], pose_cmd[2]]
        msg.speed = 5.0
        self.data_node.motion_cmd_publisher.publish(msg)
    
    def check_pos(self,pose_cmd):
        print(f"[PreciseAlignmentFSM] 檢查馬達位置: {pose_cmd}")
        # if np.allclose(self.data_node.current_pose, pose_cmd,atol=0.05):
        #     print("馬達已經到位置")
        #     return True
        # else:
        #     print("馬達尚未到位置")
        #     return False



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