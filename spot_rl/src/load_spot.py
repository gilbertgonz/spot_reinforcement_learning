from stable_baselines3 import PPO
from spot_env_one_waypoint import SpotEnv

# Create the environment
env = SpotEnv()

# Load the trained model
model = PPO.load("./tb_log/ppo_model_200000_steps.zip")

# Evaluate the agent for 10 episodes
total_reward = 0.0
episodes = 10
for i in range(episodes):
    obs = env.reset()
    done = False
    episode_reward = 0.0
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        episode_reward += reward
    print("Episode " + str(i+1) + " total reward: " + str(episode_reward))
    total_reward += episode_reward

# Calculate the mean reward over the episodes
mean_reward = total_reward / episodes
print("Mean reward: " + str(mean_reward))
