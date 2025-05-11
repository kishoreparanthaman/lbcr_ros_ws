import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/forward_velocity_controller/commands',
            10
        )
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Load joint velocity data from one RL episode
        self.velocities = np.load(
            "/home/kishore/lbcr/rl_arm/td3-robosuite-door/tmp/td3_ur5e_door/joint_velocities_episode_1.npy"
        )
        self.step = 0

    def timer_callback(self):
        if self.step >= len(self.velocities):
            self.get_logger().info('✅ Done publishing all velocities.')
            return

        raw_vel = self.velocities[self.step]
        if len(raw_vel) != 6:
            self.get_logger().warn(f"⚠️ Expected 6 joints, got {len(raw_vel)}. Truncating to 6.")
            raw_vel = raw_vel[:6]

        msg = Float64MultiArray()
        msg.data = raw_vel.tolist()
        self.publisher_.publish(msg)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
