# ~/ros2_ws/src/my_flexbe_behaviors/src/my_flexbe_behaviors/states/talk_state.py

from flexbe_core import EventState, Logger
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class TalkState(EventState):
    '''
    這個狀態會訂閱 /input_msg，並將固定訊息發布到 /output_msg。
    '''

    def __init__(self):
        super().__init__(outcomes=['done'])

        # 初始化 ROS node（僅第一次進入會觸發）
        self._node = rclpy.create_node('talk_state_node')
        self._pub = self._node.create_publisher(String, '/output_msg', 10)
        self._sub = self._node.create_subscription(String, '/input_msg', self.cb_input, 10)
        self._received_msg = None

    def cb_input(self, msg):
        self._received_msg = msg.data

    def on_enter(self, userdata):
        Logger.loginfo('等待來自 /input_msg 的資料...')

    def execute(self, userdata):
        if self._received_msg:
            Logger.loginfo(f'收到: {self._received_msg}')

            out_msg = String()
            out_msg.data = 'FlexBE 已收到你的訊息！'
            self._pub.publish(out_msg)

            return 'done'
        return None
