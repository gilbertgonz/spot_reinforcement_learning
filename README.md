# What is this?

This project aims to implement a Reinforcement Learning (RL) algorithm to enable the Boston Dynamics' Spot robot to navigate in low-gravity environments. The objective is to develop an RL agent that can learn to control the robot's movement and keep it stable in these conditions.

# Requirements

- Boston Dynamics' Spot robot (not needed but can be helpful for testing on actual hardware)
- [Spot SDK](https://github.com/boston-dynamics/spot-sdk)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation)
- [Stable Baselines 3](https://stable-baselines3.readthedocs.io/en/master/guide/install.html)
- [Open AI Gym](https://www.gymlibrary.dev/)

# Usage

Connect to the robot using the Spot SDK and launch the robot's ROS interface ('spot_ros'), if possible.

Launch the Gazebo simulation (assuming you have your workspace properly sourced):

    roslaunch spot_config gazebo.launch

Run the training script to train the RL agent using the PPO, A2C, or DQN algorithms:

    python3 train_ppo_spot.py  # we are using PPO for this example

After the agent has finished training, run the load_spot.py script to test the agent's performance:

    python3 load_spot.py

# Docker
I have created an open source docker image containing all the required files/packages. You can use the following command to pull the docker image:

    docker pull gilbertgonzalezz/spot_rl:1.0
    
Start the Docker container by running:

    # Enable access to xhost from container
    xhost +

    # Run docker and open bash shell
    docker run -it --privileged \
    --env=LOCAL_USER_ID="$(id -u)" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=:0 \
    --network host \
    --name=spot_rl_container gilbertgonzalezz/spot_rl:1.0 bash

# Acknowledgements

This project was developed as part of a research project at Florida International University (FIU). I would like to acknowledge the support provided by the Applied Research Center at FIU.
