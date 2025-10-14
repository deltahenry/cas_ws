import time
import rclpy
from rclpy.node import Node
from common_msgs.msg import Recipe

#parameters
timer_period = 0.2  # seconds


# --- ROS2 Node ---
class RecipeNode(Node):
    def __init__(self):       

        self.mode = "assembly"
        self.target_height = 150.0
        self.target_depth = 0.0
        self.target_layer = 1
        

        # 初始化 ROS2 Node
        #subscriber
        super().__init__('recipe_node')
        self.recipe_subscriber = self.create_subscription(
            Recipe,
            'recipe_cmd',
            self.recipe_callback,
            10
        )

        self.recipe_publisher = self.create_publisher(
            Recipe,
            'recipe_data',
            10
        )


    def recipe_callback(self, msg: Recipe):
        print(f"Received recipe: {msg}")
        self.mode = msg.mode 
        self.target_height = msg.height
        self.target_depth = msg.depth
        self.target_layer = msg.layer
        # 在這裡可以添加更多的處理邏輯
        # 例如，根據接收到的 recipe 更新其他狀態或觸發其他操作


def main():
    rclpy.init()
    data = RecipeNode()                 # ROS2 subscriber node

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(data)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

            # Publish the current recipe data periodically
            recipe_msg = Recipe()
            recipe_msg.mode = data.mode
            recipe_msg.height = data.target_height
            recipe_msg.depth = data.target_depth
            data.recipe_publisher.publish(recipe_msg)
            # print(f"Published recipe: {recipe_msg}")

            time.sleep(timer_period)

    except KeyboardInterrupt:
        pass
    finally:
        data.destroy_node()
        rclpy.shutdown()

# 🏁 若此檔案直接執行，就進入 main()
if __name__ == "__main__":
    main()