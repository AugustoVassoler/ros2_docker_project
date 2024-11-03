import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import time

class MemoryPublisher(Node):
    def __init__(self):
        super().__init__("memory_publisher")
        self.publisher_ = self.create_publisher(String, "memory_info", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        mem = psutil.virtual_memory()
        total_memory = mem.total / (1024 ** 3)  # Em GB
        used_memory = mem.used / (1024 ** 3)  # Em GB
        memory_percent = mem.percent
        msg = f"Total: {total_memory:.2f} GB, Used: {used_memory:.2f} GB, Percent: {memory_percent:.2f}%"
        self.publisher_.publish(String(data=msg))
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    memory_publisher = MemoryPublisher()
    rclpy.spin(memory_publisher)
    memory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()