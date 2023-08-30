#!/usr/bin/env python3
import rospy
import air_lab3.srv
import TstML
import rospkg
import TstML.Executor

import std_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import tf
import time
import math


class ExploreExecutor(TstML.Executor.AbstractNodeExecutor):
  def __init__(self, node, context):
    super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
          context)

    self.to_positionControl_pub = rospy.Publisher("to_position_control",
              std_msgs.msg.Empty, queue_size = 1)

    self.max_velocity_pub = rospy.Publisher("/husky0/max_velocity",
              geometry_msgs.msg.Twist, queue_size = 1)
    self.cmd_position_pub = rospy.Publisher("/husky0/cmd_position",
              geometry_msgs.msg.PoseStamped, queue_size = 1)

    self.position_reached_sub =  rospy.Subscriber("/husky0/point_reached",
              std_msgs.msg.Empty, self.position_reached_callback)

    self.odometry_suv = rospy.Subscriber("/husky0/odometry",
              nav_msgs.msg.Odometry, self.position_callback)

    self.explorelist = list()
    self.count = 0

    self.position = geometry_msgs.msg.Pose()

    time.sleep(2.0)

  def position_callback(self, msg):
      self.position = msg.pose.pose
      #print(msg)

  def position_reached_callback(self, res):
    print("[EXPLORE] position_reached_callback")
    if(self.count < len(self.explorelist)):
        print(self.explorelist[self.count].pose.position.x)
        print(self.explorelist[self.count].pose.position.y)
        print(self.count)
        self.cmd_position_pub.publish(self.explorelist[self.count])
    else:
        self.explorelist = list()
        self.count = 0
        self.executionFinished(TstML.Executor.ExecutionStatus.Finished())
    self.count = self.count + 1

  def start(self):
    radius = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "radius")
    a = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "a")
    b = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "b")

    print("radius - "+str(radius))
    print("A - "+str(a))
    print("B - "+str(b))

    max_veclocity = 0.5

    max_veclocity_message = geometry_msgs.msg.Twist()
    max_veclocity_message.linear.x = max_veclocity
    max_veclocity_message.linear.y = max_veclocity
    max_veclocity_message.linear.z = max_veclocity

    max_veclocity_message.angular.x = (3*max_veclocity)
    max_veclocity_message.angular.y = (3*max_veclocity)
    max_veclocity_message.angular.z = (3*max_veclocity)

    self.max_velocity_pub.publish(max_veclocity_message)

    r = 0
    t = 0
    i = 0


    while(r < radius):
        dest_point = geometry_msgs.msg.PoseStamped()
        t = (i/2) * (math.pi)
        x = (a + (b * t)) * math.cos(t)
        y = (a + (b * t)) * math.sin(t)
        r = math.sqrt(x**2+y**2)
        i = i + 1


        print("i -"+str(i))
        print("r - "+str(r))
        print("(a + b * t) - "+str(a + b * t))
        print("x - "+str(x))
        print("y - "+str(y))
        print("t - "+str(t))

        dest_point.pose.position.x = self.position.position.x + x
        dest_point.pose.position.y = self.position.position.y + y
        dest_point.pose.position.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, t+(math.pi/2))
        dest_point.pose.orientation.x = quat[0]
        dest_point.pose.orientation.y = quat[1]
        dest_point.pose.orientation.z = quat[2]
        dest_point.pose.orientation.w = quat[3]

        print("**********DEST POINT*********")
        print(dest_point)
        #self.cmd_position_pub.publish(dest_point)
        self.explorelist.append(dest_point)
        print("**********explorelist*********")
        print(self.explorelist)
    self.to_positionControl_pub.publish()
    time.sleep(1)
    self.cmd_position_pub.publish(self.explorelist[self.count])
    self.count = self.count + 1

    return TstML.Executor.ExecutionStatus.Started()
  def pause(self):
    return TstML.Executor.ExecutionStatus.Paused()
  def resume(self):
    return TstML.Executor.ExecutionStatus.Running()
  def stop(self):
    return TstML.Executor.ExecutionStatus.Finished()
  def abort(self):
    return TstML.Executor.ExecutionStatus.Aborted()
