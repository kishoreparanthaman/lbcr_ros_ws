import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class TestVelocityPublisher(Node):
    def __init__(self):
        super().__init__('test_velocity_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray,
                                                '/forward_velocity_controller/commands', 10)
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Example hardcoded joint velocities (6 DOF)
        self.test_velocities = [[0.1, 0, 0, 0, 0, 0]] * 50  # repeat same velocity for 50 steps
        self.step = 0

    def timer_callback(self):
        if self.step >= len(self.test_velocities):
            self.get_logger().info('Finished publishing test velocities.')
            return

        msg = Float64MultiArray()
        msg.data = self.test_velocities[self.step]
        self.publisher_.publish(msg)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
