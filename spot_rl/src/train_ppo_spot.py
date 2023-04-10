import gym
from stable_baselines3 import PPO
from spot_env_one_waypoint import SpotEnv
from stable_baselines3.common.callbacks import CheckpointCallback

# Save a checkpoint every 1000 steps
checkpoint_callback = CheckpointCallback(save_freq=20000, save_path="./tb_log/", name_prefix="ppo_model")

# Create the environment
env = SpotEnv()

# Instantiate the agent
model = PPO('MlpPolicy', env, verbose=1, tensorboard_log="./tb_log/", exploration_fraction=0.2, n_steps = 2048) # Monitor with: tensorboard --logdir ./tb_log/

# Train the agent for X amount of steps
model.learn(total_timesteps=200000, callback=checkpoint_callback)

print("######################## Done Training ########################")

# Save the trained model
model.save("ppo_model_final")
