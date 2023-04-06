import gym
from stable_baselines3 import A2C
from spot_env import SpotEnv
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from multiprocessing import freeze_support
import numpy as np

# Save a checkpoint every 1000 steps
checkpoint_callback = CheckpointCallback(save_freq=10000, save_path="./tb_log/", name_prefix="a2c_model")

def make_env():
    return SpotEnv()

if __name__ == "__main__":
    freeze_support()

    # Create environments
    n_envs = 4
    env = SubprocVecEnv([make_env] * n_envs)

    # Instantiate the agent
    model = A2C('MlpPolicy', env, verbose=1, tensorboard_log="./tb_log/") # Monitor with: tensorboard --logdir ./tb_log/

    # Train the agent for X amount of steps
    model.learn(total_timesteps=70000, callback=checkpoint_callback)

    print("######################## Done Training ########################")

    # Save the trained model
    model.save("a2c_model_final")
