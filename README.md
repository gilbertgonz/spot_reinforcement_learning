# RL-Based Navigation with the Spot Robot in Low Gravity Environments

This project aims to implement a Reinforcement Learning (RL) algorithm to enable the Boston Dynamics' Spot robot to navigate in low-gravity environments. The objective is to develop an RL agent that can learn to control the robot's movement and keep it stable in these conditions.

## Requirements

**The following are the hardware and software requirements needed to reproduce the results of this project:**

- Boston Dynamics' Spot robot
- Spot SDK (https://github.com/boston-dynamics/spot-sdk)
- ROS Melodic (http://wiki.ros.org/melodic/Installation)
- Stable Baselines 3 (https://stable-baselines3.readthedocs.io/en/master/guide/install.html)
- Open AI Gym (https://www.gymlibrary.dev/)

## Installation

Clone the Spot SDK repository and follow the instructions to set up the environment.

Install ROS Melodic using the instructions on the ROS Wiki.

Install Open AI Gym and Stable Baselines 3 using pip:

    pip install stable-baselines3
    pip install gym

## Usage

Connect to the robot using the Spot SDK and launch the robot's ROS interface ('spot_ros').

Launch the Gazebo simulation (assuming you have your workspace properly sourced):

    roslaunch spot_config gazebo.launch

Run the training script to train the RL agent using the PPO, A2C, or DQN algorithms:

    python3 train_ppo_spot.py  # we are using PPO for this example

After the agent has finished training, run the load_spot.py script to test the agent's performance:

    python3 load_spot.py

## Acknowledgements

This project was developed as part of a research project at Florida International University (FIU). I would like to acknowledge the support provided by the Applied Research Center at FIU.
