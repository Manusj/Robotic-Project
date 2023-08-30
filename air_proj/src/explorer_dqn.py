#!/usr/bin/env python3

import rospy
import air_lab3.srv
import rospkg

import std_msgs.msg
import visualization_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import lrs_kdb_msgs.srv
import tf
import time
import math
import random
from air_simple_sim_msgs.msg import SemanticObservation
import json
import visualize_env
import tensorflow
import dqn
from std_srvs.srv import Empty
import numpy as np

EPISODES = 30
GRID_WIDTH = 20
MAX_ITERATIONS = 50


class exploreDQN:
    def __init__(self):
        self.debug = ""
        self.episode_done = False
        self.action_size = 8
        self.action = -1
        self.batch_size = 16
        self.train_step = 0
        self.episode_count = 0
        self.episode_epsilone = 1.0
        self.reset_env_flag = False
        self.epsilon_min = 0.01
        self.error_count = 0
        self.start_position = geometry_msgs.msg.PoseStamped()
        self.start_position.pose.position.x = 0
        self.start_position.pose.position.y = 0
        self.episode_rewards = []

        self.previous_occupancyGrid = None
        self.current_occupancyGrid = None
        self.OccupancyGrid_sub =  rospy.Subscriber("/husky0/map_display", nav_msgs.msg.OccupancyGrid, self.OccupancyGrid_received)

        self.odometry_suv = rospy.Subscriber("/husky0/odometry", nav_msgs.msg.Odometry, self.position_callback)
        self.to_waypointControl_pub = rospy.Publisher("/husky0/to_waypoint_control",std_msgs.msg.Empty, queue_size = 1)
        self.to_destination_pub = rospy.Publisher("/husky0/destination",geometry_msgs.msg.PoseStamped, queue_size = 1)

        self.cmd_position_pub = rospy.Publisher("/husky0/cmd_position",geometry_msgs.msg.PoseStamped, queue_size = 1)
        self.to_positionControl_pub = rospy.Publisher("to_position_control",std_msgs.msg.Empty, queue_size = 1)
        self.position_reached_sub =  rospy.Subscriber("/husky0/point_reached",std_msgs.msg.Empty, self.position_reached_callback)

        self.max_velocity_pub = rospy.Publisher("/husky0/max_velocity",geometry_msgs.msg.Twist, queue_size = 1)
        self.cmd_waypoint_pub = rospy.Publisher("/husky0/cmd_waypoints",nav_msgs.msg.Path, queue_size = 1)

        self.waypoint_finished_sub =  rospy.Subscriber("/husky0/waypoints_finished",std_msgs.msg.Empty, self.waypoints_finished_callback)

        self.noplan_sub =  rospy.Subscriber("/husky0/noplan",std_msgs.msg.Empty, self.no_plan_generated)

        self.boundaryVisulizer = visualize_env.visualize_bountry(GRID_WIDTH)
        self.semanticVisulizer = visualize_env.visualize_semantic_objects()

        self.dqn_model = dqn.DQN(self.state_size, 8)

    def position_reached_callback(self, res):
        self.error_count = 0
        if not self.reset_env_flag:
            self.DQN_doAction()
        elif self.reset_env_flag:
            self.reset_env()


    def no_plan_generated(self, msg):
        self.error_count = self.error_count + 1
        if self.error_count >= 16:
            print("\n Timeout... reseting position")
            dest_point = geometry_msgs.msg.PoseStamped()

            dest_point.pose.position.x = round(self.start_position.pose.position.x)
            dest_point.pose.position.y = round(self.start_position.pose.position.y)
            dest_point.pose.position.z = 0

            quat = tf.transformations.quaternion_from_euler(0, 0, 0)
            dest_point.pose.orientation.x = quat[0]
            dest_point.pose.orientation.y = quat[1]
            dest_point.pose.orientation.z = quat[2]
            dest_point.pose.orientation.w = quat[3]

            self.to_positionControl_pub.publish()
            time.sleep(1)
            self.cmd_position_pub.publish(dest_point)
            return


        if not self.reset_env_flag:
            print("No plan generted - impossible to do action")
            self.train_step = self.train_step+1

            if(self.train_step > MAX_ITERATIONS):
                self.episode_done = True

            self.DQN_Fit()

            if(self.episode_count<EPISODES and not self.episode_done):
                self.DQN_doAction()


            if(self.episode_done and self.episode_count<EPISODES):
                print("Episode Done... Going to Random Point and reseting env")
                self.dqn_model.update_tensorboard(self.episode_count, self.episode_rewards, self.getScore(), self.episode_epsilone, self.dqn_model.epsilon)
                self.episode_rewards = []
                self.reset_env()
                self.episode_count = self.episode_count + 1
                self.episode_done = False
                self.train_step = 0
                self.episode_epsilone = self.episode_epsilone * 0.99
                self.dqn_model.epsilon = self.episode_epsilone
        elif self.reset_env_flag:
            time.sleep(2)
            self.reset_env()



    def position_callback(self, msg):
        self.position = msg.pose.pose




    def getScore(self):
        unexplored = self.current_occupancyGrid.data.count(-1)
        explored = len(self.current_occupancyGrid.data) - unexplored
        score = explored/len(self.current_occupancyGrid.data)
        return score


    def reward_function(self):
        grid_data_change = tuple(map(lambda i, j: i - j, self.current_occupancyGrid.data, self.previous_occupancyGrid.data))
        reward = sum(x > 0 for x in grid_data_change)
        self.previous_occupancyGrid = self.current_occupancyGrid
        neg_reward = -200
        #print("Reward - " + str(reward + neg_reward))
        return reward + neg_reward

    def extract_gridData(self, occupancyGrid):
        origin_x = occupancyGrid.info.origin.position.x
        origin_y = occupancyGrid.info.origin.position.y
        width = occupancyGrid.info.width
        resolution = 0.1
        data = list(occupancyGrid.data)

        start = int((-GRID_WIDTH - origin_y)/resolution)
        end = int((GRID_WIDTH - origin_y)/resolution)

        sliced_data = []
        for x in np.arange(-GRID_WIDTH, GRID_WIDTH,0.1):
            j = int((x - origin_x)/resolution)
            sliced_data = sliced_data + data[start+(j*width) : end+(j*width)]

        # Updating current position information
        j = int((self.position.position.x - origin_x)/resolution)
        i = int((self.position.position.y - origin_y)/resolution)

        data[j+(i*width)] = 50

        return tuple(sliced_data)

    def OccupancyGrid_received(self, occupancyGrid):
        occupancyGrid.data = self.extract_gridData(occupancyGrid)
        self.state_size = len(occupancyGrid.data)

        if(self.previous_occupancyGrid == None):
            self.previous_occupancyGrid = occupancyGrid
        self.current_occupancyGrid = occupancyGrid

    def robot_action(self, action_num):


        #print("Robot Action:"+str(action_num))
        self.action = action_num
        x = self.position.position.x
        y = self.position.position.y
        neighbours=[[x-1,y-1],[x-1,y],[x-1,y+1],[x,y-1],[x,y+1],[x+1,y-1],[x+1,y],[x+1,y+1]]

        clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

        neighbours[action_num][0] =  clamp(neighbours[action_num][0], -GRID_WIDTH, GRID_WIDTH)
        neighbours[action_num][1] =  clamp(neighbours[action_num][1], -GRID_WIDTH, GRID_WIDTH)


        dest_point = geometry_msgs.msg.PoseStamped()
        dest_point.pose.position.x = neighbours[action_num][0]
        dest_point.pose.position.y = neighbours[action_num][1]
        dest_point.pose.position.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        dest_point.pose.orientation.x = quat[0]
        dest_point.pose.orientation.y = quat[1]
        dest_point.pose.orientation.z = quat[2]
        dest_point.pose.orientation.w = quat[3]

        self.to_waypointControl_pub.publish()
        time.sleep(1)

        self.to_destination_pub.publish(dest_point)

    def reset_env(self):
        # Move to random point
        max_veclocity = 0.5
        self.reset_env_flag = True

        max_veclocity_message = geometry_msgs.msg.Twist()
        max_veclocity_message.linear.x = max_veclocity
        max_veclocity_message.linear.y = max_veclocity
        max_veclocity_message.linear.z = max_veclocity

        max_veclocity_message.angular.x = (6*max_veclocity)
        max_veclocity_message.angular.y = (6*max_veclocity)
        max_veclocity_message.angular.z = (6*max_veclocity)

        x_rand = random.randint(-GRID_WIDTH,GRID_WIDTH)
        y_rand = random.randint(-GRID_WIDTH,GRID_WIDTH)

        dest_point = geometry_msgs.msg.PoseStamped()
        dest_point.pose.position.x = x_rand
        dest_point.pose.position.y = y_rand
        dest_point.pose.position.z = 0
        self.start_position.pose.position.x = x_rand
        self.start_position.pose.position.y = y_rand

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        dest_point.pose.orientation.x = quat[0]
        dest_point.pose.orientation.y = quat[1]
        dest_point.pose.orientation.z = quat[2]
        dest_point.pose.orientation.w = quat[3]

        self.to_waypointControl_pub.publish()
        time.sleep(1)

        self.to_destination_pub.publish(dest_point)


    def DQN_doAction(self):
        state = np.reshape(self.current_occupancyGrid.data, [1, self.state_size])

        if np.random.rand() <= self.dqn_model.epsilon:
            print("Exploring action")
            action_num = random.randint(0,7)
        else:
            print("Exploiting action")
            act_values = self.dqn_model.model.predict(state)
            action_num = np.argmax(act_values[0])

        self.robot_action(action_num)

    def DQN_Fit(self):
        reward = self.reward_function()
        self.episode_rewards.append(reward)
        print("Episode: {}, Iteration: {}, Reward: {}, Action: {}, Score :{}".format(self.episode_count, self.train_step, reward, self.action, self.getScore()))
        state = np.reshape(self.previous_occupancyGrid.data, [1, self.state_size])
        next_state = np.reshape(self.current_occupancyGrid.data, [1, self.state_size])
        self.dqn_model.memorize(state, self.action, reward, next_state, self.episode_done)

        if self.episode_done:
            self.debug = self.debug + "\n" + "episode: {}/{}, score: {}, epsilon_end: {:.2}, epsilon_start: {:.2}".format(self.episode_count, EPISODES, self.getScore(), self.dqn_model.epsilon, self.episode_epsilone)
            print("episode: {}/{}, score: {}, epsilon_end: {:.2}, epsilon_start: {:.2}"
                  .format(self.episode_count, EPISODES-1, self.getScore(), self.dqn_model.epsilon, self.episode_epsilone))
            return

        if len(self.dqn_model.memory) > self.batch_size:
            if self.train_step == MAX_ITERATIONS:
                self.dqn_model.terminal_state = True
            else:
                self.dqn_model.terminal_state = False
            self.dqn_model.replay(self.batch_size)
        else:
            print("Memorizing, Mem Size - {}".format(len(self.dqn_model.memory)))

    def waypoints_finished_callback(self, req):
        self.error_count = 0
        if not self.reset_env_flag:
            self.train_step = self.train_step+1

            if(self.train_step > MAX_ITERATIONS):
                self.episode_done = True

            self.DQN_Fit()

            if(self.episode_count<EPISODES and not self.episode_done):
                self.DQN_doAction()


            if(self.episode_done and self.episode_count<EPISODES):
                print("Episode Done... Going to Random Point and reseting env")
                self.reset_env()
                self.dqn_model.update_tensorboard(self.episode_count, self.episode_rewards, self.getScore(), self.episode_epsilone, self.dqn_model.epsilon)
                self.episode_count = self.episode_count + 1
                self.episode_done = False
                self.train_step = 0
                self.episode_epsilone = self.episode_epsilone * 0.9
                self.dqn_model.epsilon = self.episode_epsilone
                if(self.dqn_model.epsilon <= self.epsilon_min):
                    self.dqn_model.epsilon = self.epsilon_min
                    self.episode_epsilone = self.epsilon_min

        elif self.reset_env_flag:
            self.reset_env_flag = False

            motionPlanService = rospy.ServiceProxy('/husky0/map_reset', Empty)
            motionPlanService()
            if(self.episode_count<EPISODES):
                self.DQN_doAction()
            else:
                self.dqn_model.save(r'/home/mansj125/firstModel.h5')
                print("\n\nTraining Report:")
                print(self.debug)




    def start(self):
        self.tf_listener = tf.TransformListener()

        max_veclocity = 0.5

        max_veclocity_message = geometry_msgs.msg.Twist()
        max_veclocity_message.linear.x = max_veclocity
        max_veclocity_message.linear.y = max_veclocity
        max_veclocity_message.linear.z = max_veclocity

        max_veclocity_message.angular.x = (6*max_veclocity)
        max_veclocity_message.angular.y = (6*max_veclocity)
        max_veclocity_message.angular.z = (6*max_veclocity)

        self.max_velocity_pub.publish(max_veclocity_message)

        self.DQN_doAction()
