import gym
from stable_baselines3 import A2C
from spot_env_one_waypoint import SpotEnv
from stable_baselines3.common.callbacks import CheckpointCallback

# Save a checkpoint every 1000 steps
checkpoint_callback = CheckpointCallback(save_freq=50000, save_path="./tb_log/", name_prefix="a2c_model")

# Create environment
env = SpotEnv()

# Instantiate the agent
model = A2C('MlpPolicy', env, verbose=1, tensorboard_log="./tb_log/") # Monitor with: tensorboard --logdir ./tb_log/

# Train the agent for X amount of steps
model.learn(total_timesteps=500000, callback=checkpoint_callback)

print("######################## Done Training ########################")

# Save the trained model
model.save("a2c_model_final")
