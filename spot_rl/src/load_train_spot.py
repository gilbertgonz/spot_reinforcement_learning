import gym
from stable_baselines3 import PPO
from spot_env_waypoints import SpotEnv
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback

# Save a checkpoint every 1000 steps
checkpoint_callback = CheckpointCallback(save_freq=10000, save_path="./tb_log/", name_prefix="ppo_model")

# Create a new environment for additional training
env = DummyVecEnv([lambda: SpotEnv()])

# Load saved model
model = PPO.load("./tb_log/ppo_new.zip", env=env, tensorboard_log="./tb_log/")

# Continue training for additional 10000 timesteps
model.learn(total_timesteps=500000, callback=checkpoint_callback)

print("######################## Done Training ########################")

model.save("ppo_final_upgraded")