from spot_env_waypoints import SpotEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

# Save a checkpoint every 1000 steps
checkpoint_callback = CheckpointCallback(
    save_freq=20000, 
    save_path="./tb_log/", 
    name_prefix="ppo_third")

# Log evaluation metrics to Tensorboard
eval_callback = EvalCallback(
    eval_env=DummyVecEnv([lambda: SpotEnv()]),
    callback_on_new_best=None,
    eval_freq=2000,
    n_eval_episodes=20,
    log_path="./tb_log/",
    best_model_save_path=None,
    deterministic=True,
)

# Create a new environment for additional training
env = DummyVecEnv([lambda: SpotEnv()])

# Load saved model
pretrained_model = PPO.load("./tb_log/ppo_second_80000_steps.zip", env=env)

# Freeze the first two layers of the neural network
for i, param in enumerate(pretrained_model.policy.parameters()):
    if i < 6:
        param.requires_grad = False

# Continue training for additional 10000 timesteps
pretrained_model.learn(total_timesteps=200000, callback=[checkpoint_callback, eval_callback])

print("######################## Done Training ########################")

pretrained_model.save("ppo_final")