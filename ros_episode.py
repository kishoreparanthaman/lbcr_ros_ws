import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from td3_torch import Agent  # your trained TD3 class

class TD3RealUR5e(Node):
    def __init__(self):
        super().__init__('td3_real_ur5e_node')

        # ROS publishers/subscribers
        self.vel_pub = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # TD3 Agent
        self.agent = Agent(
            actor_learning_rate=0.001,
            critic_learning_rate=0.001,
            tau=0.005,
            input_dims=(12,),  # 6 joint pos + 6 vel
            env=None,
            n_actions=6,
            layer1_size=256,
            layer2_size=128,
            batch_size=128
        )
        self.agent.load_models()
        self.joint_obs = None

        # Call control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_callback)

    def joint_state_callback(self, msg):
        joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        pos = []
        vel = []
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity))
        for j in joint_names:
            pos.append(name_to_pos.get(j, 0.0))
            vel.append(name_to_vel.get(j, 0.0))
        self.joint_obs = np.array(pos + vel, dtype=np.float32)

    def control_callback(self):
        if self.joint_obs is None:
            return

        action = self.agent.choose_action(self.joint_obs, validation=True)
        action = np.clip(action, -0.2, 0.2)

        msg = Float64MultiArray()
        msg.data = action.tolist()
        self.vel_pub.publish(msg)

def main():
    rclpy.init()
    node = TD3RealUR5e()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
