import gym
from stable_baselines3 import DQN
from spot_env import SpotEnv
from stable_baselines3.common.callbacks import CheckpointCallback

# Save a checkpoint every 1000 steps
checkpoint_callback = CheckpointCallback(save_freq=50000, save_path="./tb_log/", name_prefix="dqn_model")

# Create environment
env = SpotEnv()

# Instantiate the agent
model = DQN('MlpPolicy', env, verbose=1, tensorboard_log="./tb_log/") # Monitor with: tensorboard --logdir ./tb_log/

# Train the agent for X amount of steps
model.learn(total_timesteps=500000, callback=checkpoint_callback)

print("######################## Done Training ########################")

# Save the trained model
model.save("dqn_model_final")
