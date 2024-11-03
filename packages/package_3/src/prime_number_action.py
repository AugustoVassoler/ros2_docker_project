import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class PrimeNumberAction(Node):
    def __init__(self):
        super().__init__("prime_number_action")
        self._action_server = ActionServer(
            self,
            Fibonacci,
            "find_nth_prime",
            self.execute_callback)

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        nth_prime = 0
        count = 0
        num = 1
        primes = []

        while count < goal_handle.request.order:
            num += 1
            if self.is_prime(num):
                count += 1
                primes.append(num)
                feedback_msg.sequence = primes
                goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = primes
        return result

    def is_prime(self, num):
        if num < 2:
            return False
        for i in range(2, int(num**0.5) + 1):
            if num % i == 0:
                return False
        return True

def main(args=None):
    rclpy.init(args=args)
    prime_number_action = PrimeNumberAction()
    rclpy.spin(prime_number_action)
    prime_number_action.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()