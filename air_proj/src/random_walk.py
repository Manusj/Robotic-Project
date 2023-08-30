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

from std_srvs.srv import Empty

class randomWalk:
    def __init__(self):
        self.to_waypointControl_pub = rospy.Publisher("/husky0/to_waypoint_control",std_msgs.msg.Empty, queue_size = 1)
        self.to_destination_pub = rospy.Publisher("/husky0/destination",geometry_msgs.msg.PoseStamped, queue_size = 1)

        self.max_velocity_pub = rospy.Publisher("/husky0/max_velocity",geometry_msgs.msg.Twist, queue_size = 1)
        self.cmd_waypoint_pub = rospy.Publisher("/husky0/cmd_waypoints",nav_msgs.msg.Path, queue_size = 1)

        self.waypoint_finished_sub =  rospy.Subscriber("/husky0/waypoints_finished",std_msgs.msg.Empty, self.waypoints_finished_callback)

        self.semanticVisulizer = visualize_env.visualize_semantic_objects()
        self.forntierVisulizer = visualize_env.visualize_frontier()



    def waypoints_finished_callback(self, req):
        self.goto_randomPoint()

    def goto_randomPoint(self):
        x_rand = random.randint(-35,35)
        y_rand = random.randint(-35,35)

        dest_point = geometry_msgs.msg.PoseStamped()
        dest_point.pose.position.x = x_rand
        dest_point.pose.position.y = y_rand
        dest_point.pose.position.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        dest_point.pose.orientation.x = quat[0]
        dest_point.pose.orientation.y = quat[1]
        dest_point.pose.orientation.z = quat[2]
        dest_point.pose.orientation.w = quat[3]

        self.to_waypointControl_pub.publish()
        time.sleep(1)

        self.to_destination_pub.publish(dest_point)


    def start(self):
        self.tf_listener = tf.TransformListener()

        max_veclocity = 0.5

        max_veclocity_message = geometry_msgs.msg.Twist()
        max_veclocity_message.linear.x = max_veclocity
        max_veclocity_message.linear.y = max_veclocity
        max_veclocity_message.linear.z = max_veclocity

        max_veclocity_message.angular.x = (3*max_veclocity)
        max_veclocity_message.angular.y = (3*max_veclocity)
        max_veclocity_message.angular.z = (3*max_veclocity)

        self.max_velocity_pub.publish(max_veclocity_message)
        self.goto_randomPoint()
