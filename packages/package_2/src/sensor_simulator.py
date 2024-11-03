import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64
import random
from collections import deque

class SensorSimulator(Node):
    def __init__(self):
        super().__init__("sensor_simulator")
        self.publisher_ = self.create_publisher(Float64, "sensor_data", 10)
        self.service_reset = self.create_service(Trigger, "reset_filter", self.reset_filter_callback)
        self.service_last_values = self.create_service(Trigger, "get_last_values", self.get_last_values_callback)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.values = deque(maxlen=5)

    def timer_callback(self):
        sensor_value = random.uniform(0.0, 100.0)
        self.values.append(sensor_value)
        avg_value = sum(self.values) / len(self.values)
        self.publisher_.publish(Float64(data=avg_value))
        self.get_logger().info(f"Published sensor value: {avg_value}")

    def reset_filter_callback(self, request, response):
        self.values.clear()
        response.success = True
        response.message = "Filter reset successfully."
        return response

    def get_last_values_callback(self, request, response):
        response.success = True
        response.message = str(list(self.values))
        return response

def main(args=None):
    rclpy.init(args=args)
    sensor_simulator = SensorSimulator()
    rclpy.spin(sensor_simulator)
    sensor_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()