import gym
import logging
import random
from spot_env_waypoints import SpotEnv
import time

gym.logger.set_level(logging.CRITICAL)

# 1. Launch Gazebo Simulation
# 2. Run this file (or run the training file *fingers crossed*)

env = SpotEnv()

for i in range(10):
    obs = env.reset()
    done = False
    total_reward = 0.0
    time.sleep(1)
    while not done:
        action = env.action_space.sample()  # choose a random action
        obs, reward, done, info = env.step(action)
        total_reward += reward
    print("Episode " + str(i+1) + " total reward: " + str(total_reward))
