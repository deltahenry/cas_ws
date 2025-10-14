import time
import matplotlib.pyplot as plt
from transitions import Machine
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32MultiArray,Int32
from common_msgs.msg import Recipe,CurrentPose
import socket
import csv
from datetime import datetime
import os
import copy

# parameters
timer_period = 0.1  # seconds


# --- ROS2 Node ---
class DataNode(Node):
    def __init__(self):
        super().__init__('data_node')

        # CSV 檔案初始化
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file_path = '/home/henry/cas_ws/src/logs/' + timestamp + '.csv'
        self.csv_file = open(self.csv_file_path, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow(["Timestamp",
                                "Layer",
                                "DataType",
                                "X",
                                "Y",
                                "Z",
                                "Yaw",
                                "Roll",
                                "Pitch"])

        self.detection_cmd = None
        self.conn = None
        self.addr = None
        self.compensate_x = 0.0
        self.compensate_z = 0.0

        self.golden_x = 84.6
        self.golden_z = -77.6

        # self.golden_layer = '5' # 0~9層
        self.golden_z_modify_list = [0.0, 0.0, 10.0, 2.0, 0.0, 2.0, 10.0, 0.0, 0.0] # 1~9層

        #mecheye 變數
        self.current_x_queue = [0.0,0.0,0.0]
        self.current_z_queue = [0.0,0.0,0.0]

        self.current_x = 0.0
        self.current_z = 0.0

        #robot 變數
        self.current_pose = [0.0,0.0,0.0] #x y yaw(rad)
        self.current_height = 0

        #target 
        self.target_layer = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0 #deg
        self.target_z = 0.0

        self.init_tcp()

        self.dection_cmd_subscriber = self.create_subscription(
            String,
            'detection_cmd',
            self.dection_cmd_callback,
            10
        )

        self.recipe_data_subscriber = self.create_subscription(
            Recipe,
            'recipe_cmd',
            self.recipe_callback,
            10
        )

        self.compensate_value_subscriber = self.create_subscription(
            Float32MultiArray,      
            'ui_compensate_value',
            self.compensate_value_callback,
            10
        )

        # self.target_pose_subscriber = self.create_subscription(
        #     Float32MultiArray,      
        #     'ui_target_pose',
        #     self.target_pose_callback,
        #     10
        # )

        self.compensate_pose_pub = self.create_publisher(
            Float32MultiArray,
            '/compensate_pose_cmd',
            10
        )

        self.current_pose_subscriber = self.create_subscription(    
            CurrentPose,
            'current_pose',
            self.current_pose_callback,
            10
        )

        self.height_info_subscriber = self.create_subscription(
            Int32,
            'lr_distance',
            self.height_info_callback,
            10
        )


    def init_tcp(self):
        HOST = "0.0.0.0"
        PORT = 8000
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允許重用 port
        self.server.bind((HOST, PORT))
        self.server.listen(1)
        self.server.setblocking(False)  # 非阻塞 accept
        print(f"[TCP] Server listening on {HOST}:{PORT}")

    def dection_cmd_callback(self, msg: String):
        self.detection_cmd = msg.data
        print(f"[DataNode] 接收到 dection_cmd: {self.detection_cmd}")

    def current_pose_callback(self, msg: CurrentPose):
        self.current_pose[0] = float(msg.pose_data[0]) # x
        self.current_pose[1] = float(msg.pose_data[1]) # y
        self.current_pose[2] = float(msg.pose_data[2])*57.2958 # yaw (deg)
        # print(f"[DataNode] 接收到 current_pose: {self.current_pose}")
        self.target_x = copy.deepcopy(self.current_pose[0])
        self.target_y = copy.deepcopy(self.current_pose[1])
        self.target_yaw = copy.deepcopy(self.current_pose[2])

    # def target_pose_callback(self, msg: Float32MultiArray):
    #     if len(msg.data) >= 4:
    #         self.target_x = msg.data[0]
    #         self.target_y = msg.data[1]
    #         self.target_yaw = msg.data[2] # deg
    #         self.target_z = msg.data[3]
    #         # print(f"[DataNode] 接收到 target_pose: x={self.target_x}, y={self.target_y}, yaw={self.target_yaw}, z={self.target_z}")
    #     else:
    #         print("[DataNode] target_pose 資料格式錯誤")

    def height_info_callback(self, msg: Int32):
        self.current_height = msg.data
        self.target_z = float(self.current_height)
        # print(f"[DataNode] 接收到 height_info: {self.current_height}")

    def compensate_value_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            mark = 'Current_Pose'                
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            self.csv_writer.writerow([timestamp,
                                      self.target_layer,
                                      mark,
                                      self.current_pose[0],
                                      self.current_pose[1],
                                      self.current_height,
                                      self.current_pose[2],
                                      "None",
                                      "None"])
                                      

            print(f"[DataNode] 接收到 ui_compensate_value: {msg.data}")
            x_value = msg.data[0]
            y_value = msg.data[1]
            yaw_value = msg.data[2]
            z_value = msg.data[3]

            mark = 'Unknown_Type'

            if y_value == -999.0 and yaw_value == -999.0 and z_value == -999.0:
                mark = 'X_Compensate'                
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                self.csv_writer.writerow([timestamp,
                                        self.target_layer,
                                        mark,
                                        x_value,
                                        "None",
                                        "None",
                                        "None",
                                        "None",
                                        "None"])
                # self.csv_writer.writerow([timestamp,mark,x_value])
                self.target_x = self.target_x + x_value

            elif x_value == -999.0 and y_value == -999.0 and z_value == -999.0:
                mark = 'Yaw_Compensate'                
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                self.csv_writer.writerow([timestamp,
                                        self.target_layer,
                                        mark,
                                        "None",
                                        "None",
                                        "None",
                                        yaw_value,
                                        "None",
                                        "None"])
                # self.csv_writer.writerow([timestamp,mark,yaw_value])
                self.target_yaw = self.target_yaw + yaw_value
            
            elif x_value == -999.0 and y_value == -999.0 and yaw_value == -999.0:
                mark = 'Z_Compensate'                
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                self.csv_writer.writerow([timestamp,
                                        self.target_layer,
                                        mark,
                                        "None",
                                        "None",
                                        z_value,
                                        "None",
                                        "None",
                                        "None"])
                # self.csv_writer.writerow([timestamp,mark,z_value])
                self.target_z = self.target_z + z_value

            mark = 'Target_Pose'                
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            self.csv_writer.writerow([timestamp,
                                      self.target_layer,
                                      mark,
                                      self.target_x,
                                      self.target_y,
                                      self.target_z,
                                      self.target_yaw,
                                      "None",
                                      "None"])
            # self.csv_writer.writerow([timestamp,mark,[self.target_x,self.target_y,self.target_yaw,self.target_z]])

        else:
            print("[DataNode] ui_compensate_value 資料格式錯誤")

    def recipe_callback(self, msg: Recipe):
        # print(f"[DataNode] 接收到 recipe_data: layer={msg.layer}")
        self.target_layer = str(msg.layer)
        print(f"[DataNode] 更新 target_layer: {self.target_layer}")

class MecheyeState(Enum):
    IDLE = "idle"
    START_DECECT = "start_detect"
    WAIT_DETECT = "wait_detect"
    CALCULATE = "calculate"
    DONE = "done"
    STOP = "stop"
    FAIL = "fail"


class Mecheye(Machine):
    def __init__(self, data_node: DataNode):

        self.phase = MecheyeState.IDLE
        self.data_node = data_node
        self.count = 0
        self.COUNT_THRESHOLD = 3

        states = [state.value for state in MecheyeState]
        transitions = [
            {"trigger": "start_detect", "source": MecheyeState.IDLE.value, "dest": MecheyeState.START_DECECT.value},
            {"trigger": "wait_detect", "source": MecheyeState.START_DECECT.value, "dest": MecheyeState.WAIT_DETECT.value},
            {"trigger": "retry", "source": MecheyeState.WAIT_DETECT.value, "dest": MecheyeState.START_DECECT.value},
            {"trigger": "calculate", "source": MecheyeState.WAIT_DETECT.value, "dest": MecheyeState.CALCULATE.value},
            {"trigger": "detect_again", "source": MecheyeState.CALCULATE.value, "dest": MecheyeState.START_DECECT.value},
            {"trigger": "done", "source": MecheyeState.CALCULATE.value, "dest": MecheyeState.DONE.value},
            {"trigger": "stop", "source": "*", "dest": MecheyeState.STOP.value},
            {"trigger": "fail", "source": "*", "dest": MecheyeState.FAIL.value},
            {"trigger": "return_to_idle", "source": "*", "dest": MecheyeState.IDLE.value},
        ]

        self.machine = Machine(model=self, states=states, transitions=transitions,
                               initial=self.phase.value,
                               auto_transitions=False, after_state_change=self._update_phase)
        
    def _update_phase(self):
        self.phase = MecheyeState(self.state)

    def step(self):
        if self.data_node.detection_cmd == "start_detect":
            self.run()
        else:
            self.reset_parameters()  # 重置參數
            self.return_to_idle()  # 返回到空閒狀態
            self.run()
            return

    def run(self):
        if self.state == MecheyeState.IDLE.value:
            if self.data_node.detection_cmd == "start_detect":
                print("[Mecheye] 接收到 start_detect 指令，開始檢測")
                self.start_detect()
            else:
                print("[Mecheye] 狀態: IDLE")

        elif self.state == MecheyeState.START_DECECT.value:
            print("[Mecheye] 狀態: START_DETECT")
            if not self.data_node.conn:
                try:
                    conn, addr = self.data_node.server.accept()
                    self.data_node.conn = conn
                    self.data_node.addr = addr
                    print(f"[Server] Connection from {addr}")
                except BlockingIOError:
                    print("[Server] No incoming connections yet.")
                    return  # 等下一輪

            try:
                self.data_node.conn.send(b"00")  # 發送開始檢測命令
                print("[Server] Sent: 00")
                self.wait_detect()
            except Exception as e:
                print(f"[Server Error] {e}")
                self.fail()

        elif self.state == MecheyeState.WAIT_DETECT.value:
            print("[Mecheye] 狀態: WAIT_DETECT")
            conn = self.data_node.conn
            if conn:
                try:
                    data = conn.recv(1024).decode()
                    # 用逗號分隔，轉成 list
                    data_list = data.split(",")

                    float_values = [float(x) for x in data_list]

                    if len(float_values)>=2:
                        self.count += 1
                        print(f"[Server] Received: {data}")
                        self.data_node.current_x = float_values[1]
                        self.data_node.current_z = float_values[2]
                        conn.send(b"01")
                        print("[Server] Sent: 01")

                        
                        # 寫入 CSV 文件
                        # timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                        # mark = 'Raw_data'
                        # self.data_node.csv_writer.writerow([timestamp,mark,data_list])
                        mark = 'Raw_Data'                
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                        self.data_node.csv_writer.writerow([timestamp,
                                                self.data_node.target_layer,
                                                mark,
                                                float_values[1],
                                                "None",
                                                float_values[2],
                                                float_values[3],
                                                float_values[4],
                                                float_values[5]])

                        print(f"[Mecheye] 收集資料中: {self.count}/3")
                        self.data_node.current_x_queue[self.count-1] = self.data_node.current_x
                        self.data_node.current_z_queue[self.count-1] = self.data_node.current_z

                        self.calculate()

                    elif len(float_values)==1:
                        print("[Server] No valid data received, retrying...")
                        self.retry()

                    else:
                        print("[Server] Invalid data format, failing...")
                        self.fail()

                except BlockingIOError:
                    pass  # 非阻塞 recv，無資料可讀

        elif self.state == MecheyeState.CALCULATE.value:
            print("[Mecheye] 狀態: CALCULATE")
            if self.count < self.COUNT_THRESHOLD:
                self.detect_again()

            else:
                x_modify,z_modify = self.data_process(self.data_node.current_x_queue,self.data_node.current_z_queue)

                # 寫入 CSV 文件
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                mark = 'Filter_Data'
                self.data_node.csv_writer.writerow([timestamp,
                                        self.data_node.target_layer,
                                        mark,
                                        x_modify,
                                        "None",
                                        z_modify,
                                        "None",
                                        "None",
                                        "None"])

                golden_z_modify = self.modify_golden_z(int(self.data_node.target_layer)-1)
                self.data_node.compensate_x = -(x_modify - self.data_node.golden_x)
                # self.data_node.compensate_z = (z_modify - self.data_node.golden_z)*0.5
                self.data_node.compensate_z = (z_modify - golden_z_modify)*0.5
                print(f"[Mecheye] 計算結果: compensate_x={self.data_node.compensate_x}, compensate_z={self.data_node.compensate_z}")

                # 寫入 CSV 文件
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                mark = 'Golden_Value'
                self.data_node.csv_writer.writerow([timestamp,
                                        self.data_node.target_layer,
                                        mark,
                                        self.data_node.golden_x,
                                        "None",
                                        golden_z_modify,
                                        "None",
                                        "None",
                                        "None"])

                # 寫入 CSV 文件
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                mark = 'Compensate_Value'
                self.data_node.csv_writer.writerow([timestamp,
                                        self.data_node.target_layer,
                                        mark,
                                        self.data_node.compensate_x,
                                        "None",
                                        self.data_node.compensate_z,
                                        "None",
                                        "None",
                                        "None"])

                msg = Float32MultiArray()
                msg.data = [self.data_node.compensate_x, self.data_node.compensate_z]
                self.data_node.compensate_pose_pub.publish(msg)
                print("[Mecheye] Published compensate_pose_cmd")
                self.done()

        elif self.state == MecheyeState.DONE.value:
            print("[Mecheye] 狀態: DONE")
            self.reset_parameters()
            self.return_to_idle()

        elif self.state == MecheyeState.STOP.value:
            print("[Mecheye] 狀態: STOP")
            self.return_to_idle()

        elif self.state == MecheyeState.FAIL.value:
            print("[Mecheye] 狀態: FAIL")
            self.reset_parameters
            self.return_to_idle()

    def reset_parameters(self):
        self.data_node.detection_cmd = None
        self.data_node.current_x_queue = [0.0,0.0,0.0]
        self.data_node.current_z_queue = [0.0,0.0,0.0]
        self.count = 0

    def remove_outlier(self, values):
        """移除一筆離群值，如果沒有明顯離群就回傳原始值"""
        if len(values) <= 2:
            return values  # 不足三筆，無法判斷 outlier

        diffs = []
        n = len(values)
        for i in range(n):
            diff_sum = sum(abs(values[i] - values[j]) for j in range(n) if j != i)
            diffs.append(diff_sum)

        outlier_index = diffs.index(max(diffs))

        # 判斷是不是明顯 outlier
        if diffs[outlier_index] < (sum(diffs) / len(diffs)) * 1.5:
            return values
        else:
            return [v for i, v in enumerate(values) if i != outlier_index]

    def data_process(self, x_queue, z_queue):
        """對 X 與 Z 的 queue 進行 outlier 過濾，回傳平均值"""
        # filtered_x = self.remove_outlier(x_queue)
        # filtered_z = self.remove_outlier(z_queue)
        filtered_x = x_queue
        filtered_z = z_queue

        x_mean = sum(filtered_x) / len(filtered_x)
        z_mean = sum(filtered_z) / len(filtered_z)

        print(f"[Mecheye] 過濾後 X: {filtered_x}, Z: {filtered_z}")
        return x_mean, z_mean

    def modify_golden_z(self, target_layer):
        """根據目標層數，調整 golden_z 的值"""
        try:
            layer_index = int(target_layer)
            if 0 <= layer_index < len(self.data_node.golden_z_modify_list):
                z_modify = self.data_node.golden_z_modify_list[layer_index]
                modified_golden_z = self.data_node.golden_z + z_modify
                print(f"[Mecheye] 目標層數: {target_layer}, 調整後 golden_z: {modified_golden_z}")
                return modified_golden_z
            else:
                print(f"[Mecheye] 目標層數 {target_layer} 超出範圍，使用預設 golden_z")
                return self.data_node.golden_z
        except ValueError:
            print(f"[Mecheye] 目標層數格式錯誤: {target_layer}，使用預設 golden_z")
            return self.data_node.golden_z

def main():
    rclpy.init()
    data = DataNode()
    system = Mecheye(data)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            system.step()
            time.sleep(timer_period)

    except KeyboardInterrupt:
        pass

    finally:
        data.destroy_node()
        data.csv_file.close()  # 關閉 CSV 檔案
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()