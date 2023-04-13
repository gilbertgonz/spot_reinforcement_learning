import gym
from gym import spaces
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
import numpy as np
import time
import random

## TODO: try training with NEAT algo once DQN and PPO is properly trained

class SpotEnv(gym.Env):
    def __init__(self):
        rospy.init_node('my_env', anonymous=True)

        # Define observation space and action space
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(520,), dtype=np.float32) # dtype=np.float32
        self.action_space = spaces.Discrete(3)

        # Create ROS publishers and subscribers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.robot_state = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)

        # Initialize variables
        self.observation = np.zeros((520,))
        self.max_episode_length = 30.0  # maximum episode length in seconds
        self.timer = None  # timer object
        self.total_reward = 0.0
        self.reward = 0.0
        self.action = None
        self.ranges = []
        self.waypoints = [(3.0113, -2.8049), (3.524, 2.096)]
        self.waypoint_position = 3.524, 2.096
        self.distance_from_goal = 0.0

        self.too_far = False
        self.done = False
        self.collision = False
        self.tipped_over = False
        self.distance_achieved = False
        self.time_over = False

    def scan_callback(self, data):
        ranges = np.array(data.ranges)
        ranges_len = len(ranges)
        range_max = data.range_max

        for i in range(len(ranges)):
            if ranges[i] == float("inf"):
                ranges[i] = range_max
        self.ranges = tuple(ranges)

    def imu_callback(self, data):
        # Check if robot is tipped over
        if abs(data.orientation.x) > 0.6 or abs(data.orientation.y) > 0.6:
            self.tipped_over = True

    def state_callback(self, data):
        # Get the index of Spot's position in the message
        spot_idx = data.name.index('/')

        # Get the x and y position of Spot
        x = data.pose[spot_idx].position.x
        y = data.pose[spot_idx].position.y

        # Save the position as a tuple
        self.robot_position = (x, y)

    def reset(self):
        # Reset simulation and get initial observation
        rospy.wait_for_service('/gazebo/reset_simulation')
        reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_simulation()
        
        # start the timer
        self.timer = time.monotonic()

        # Re-initialize variables
        self.observation = np.zeros((520,))
        self.observation = self.get_observation()

        self.total_reward = 0.0
        self.reward = 0.0

        self.distance_from_goal = 0.0
        self.waypoint_position = random.choice([wp for wp in self.waypoints if wp != self.waypoint_position])
        print("goal", str(self.waypoint_position))
        self.ranges = []
        self.action = None
        self.done = False
        self.collision = False
        self.tipped_over = False
        self.distance_achieved = False
        self.time_over = False
        self.too_far = False

        return self.observation

    def step(self, action):
        # Send control command and calculate new observation, reward, and done flag
        vel_cmd = Twist()
        if action == 0:  # Forward
            vel_cmd.linear.x = 0.4
            vel_cmd.angular.z = 0.0
            self.reward += 0.5
        elif action == 1:  # Right
            vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = -0.4
        elif action == 2:  # Left
            vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = 0.4
        # elif action == 3:  # Stop
        #     vel_cmd.linear.x = 0.0
        #     vel_cmd.angular.z = 0.0
            # self.reward -= 1.0
        # elif action == 4:  # Reverse
        #     vel_cmd.linear.x = -0.3
        #     vel_cmd.angular.z = 0.0
            # self.reward -= 2.0

        # Publish velocity command
        self.vel_pub.publish(vel_cmd)

        # check if time limit has been exceeded
        elapsed_time = time.monotonic() - self.timer
        if elapsed_time >= self.max_episode_length:
            self.time_over = True
    
        # Wait for observation to be updated
        rospy.sleep(0.1)

        self.total_reward += self.reward
        
        # Update observation, reward, and done flag
        self.observation = self.get_observation()
        self.reward = self.calculate_reward()
        self.done = self.check_done()
        
        return self.observation, self.reward, self.done, {}

    def get_observation(self):
        # Get current observation
        imu_data = rospy.wait_for_message('/imu/data', Imu)

        # Adding data to observation vector
        self.observation[0:len(self.ranges[int(len(self.ranges)/4):int(len(self.ranges)*3/4)])] = self.ranges[int(len(self.ranges)/4):int(len(self.ranges)*3/4)]
        self.observation[-5] = imu_data.orientation.x
        self.observation[-4] = self.robot_position[0]
        self.observation[-3] = self.robot_position[1]
        self.observation[-2] = self.waypoint_position[0]
        self.observation[-1] = self.waypoint_position[1]  

        for i, value in enumerate(self.ranges[int(len(self.ranges)//4):int(len(self.ranges)*3//4)]):
            if value < 0.45:
                self.collision = True
        
        return self.observation

    def calculate_reward(self):
        bad_reward = 0
        arrival_reward = 0
        
        # Calculating the pythagorean distance to the goal position
        self.distance_from_goal = np.sqrt((self.waypoint_position[0] - self.robot_position[0])**2 + (self.waypoint_position[1] - self.robot_position[1])**2)
        
        if self.distance_from_goal < 1.0:
            arrival_reward = 100
            self.distance_achieved = True
        elif self.distance_from_goal > 6.0:
            arrival_reward = -50
            self.too_far = True
        elif self.tipped_over:
            bad_reward = -40
        elif self.collision:  
            bad_reward = -40
        elif self.time_over and self.distance_from_goal > 3.0:
            print("time far")
            bad_reward = -30
        elif self.time_over and self.distance_from_goal < 3.0:
            print("time close")
            bad_reward = 10
        
        self.reward = ((3.0 - self.distance_from_goal) * 1.2 + arrival_reward + bad_reward)

        return self.reward

    def check_done(self):
        # Check if episode is done
        if self.tipped_over:
            print("tipped")
            return True
        
        if self.collision: 
            print("collided")
            return True
        
        if self.time_over:
            return True
        
        if self.distance_achieved:
            print("arrived")
            return True
        
        if self.too_far:
            print("too far")
            return True

        return False