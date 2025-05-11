import numpy as np
import os
import gym
import robosuite as suite
from robosuite.wrappers import GymWrapper
from td3_torch import Agent
import time

if __name__ == '__main__':
    save_dir = "tmp/td3_ur5e_door"
    os.makedirs(save_dir, exist_ok=True)

    env_name = "Door"

    env = suite.make(
        env_name,
        robots=["UR5e"],
        controller_configs=suite.load_controller_config(default_controller="JOINT_VELOCITY"),
        has_renderer=True,
        use_camera_obs=False,
        horizon=1000,
        render_camera="frontview",
        has_offscreen_renderer=True,
        reward_shaping=True,
        control_freq=20,
    )

    env = GymWrapper(env)

    actor_learning_rate = 0.001
    critic_learning_rate = 0.001
    batch_size = 128
    layer1_size = 256
    layer2_size = 128

    agent = Agent(
        actor_learning_rate=actor_learning_rate,
        critic_learning_rate=critic_learning_rate,
        tau=0.005,
        input_dims=env.observation_space.shape,
        env=env,
        n_actions=env.action_space.shape[0],
        layer1_size=layer1_size,
        layer2_size=layer2_size,
        batch_size=batch_size
    )

    n_games = 10
    agent.load_models()

    for episode in range(n_games):
        observation = env.reset()
        done = False
        score = 0
        joint_velocities = []

        while not done:
            action = agent.choose_action(observation, validation=True)
            joint_velocities.append(action)
            next_observation, reward, done, info = env.step(action)
            env.render()
            score += reward
            observation = next_observation
            time.sleep(0.03)

        joint_velocities = np.array(joint_velocities)
        np.save(os.path.join(save_dir, f"joint_velocities_episode_{episode}.npy"), joint_velocities)
        print(f"Episode: {episode}, score: {score}")
