#!/usr/bin/env python3

import math
# rosoy is the main API for ROS
import rospy
import random_walk
import explorer_dqn

class explore:
    def __init__(self, type):
        self.exploration_type = type

    def start_explore(self):
        if(self.exploration_type == "random-walk"):
            self.randomWalk_exploer = random_walk.randomWalk()
            self.randomWalk_exploer.start()
        elif(self.exploration_type == "dqn"):
            self.dqn_exploreer = explorer_dqn.exploreDQN()
            self.dqn_exploreer.start()
            pass
    def random_walk(self):
        pass



if __name__ == '__main__':
    # Initialise the ROS sub system
    rospy.init_node('explore', anonymous=False)
    # Create an instance of our controller
    exploration_type = rospy.get_param('~type', 'random-walk')
    ec = explore(exploration_type)
    ec.start_explore()
    # Start listening to messages and loop forever
    rospy.spin()
