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


class DriveToExecutor(TstML.Executor.AbstractNodeExecutor):
  def __init__(self, node, context):
    super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
          context)

    self.to_waypointControl_pub = rospy.Publisher("/husky0/to_waypoint_control",
              std_msgs.msg.Empty, queue_size = 1)
    self.to_destination_pub = rospy.Publisher("/husky0/destination",
              geometry_msgs.msg.PoseStamped, queue_size = 1)

    self.to_positionControl_pub = rospy.Publisher("/husky0/to_position_control",
              std_msgs.msg.Empty, queue_size = 1)

    self.max_velocity_pub = rospy.Publisher("/husky0/max_velocity",
              geometry_msgs.msg.Twist, queue_size = 1)
    self.cmd_waypoint_pub = rospy.Publisher("/husky0/cmd_waypoints",
              nav_msgs.msg.Path, queue_size = 1)
    self.cmd_position_pub = rospy.Publisher("/husky0/cmd_position",
              geometry_msgs.msg.PoseStamped, queue_size = 1)

    self.waypoint_finished_sub =  rospy.Subscriber("/husky0/waypoints_finished",
              std_msgs.msg.Empty, self.waypoints_finished_callback)

    self.position_reached_sub =  rospy.Subscriber("/husky0/point_reached",
              std_msgs.msg.Empty, self.position_reached_callback)
    time.sleep(2.0)



    #self.pub = rospy.Publisher(node.getParameter(
    #      TstML.TSTNode.ParameterType.Specific,
#          "topic"), std_msgs.msg.String, queue_size=1)
#    self.count = 0
#    self.paused = False
  def waypoints_finished_callback(self, req):
    print("[DRIVE TO] waypoints_finished_callback")
    self.executionFinished(TstML.Executor.ExecutionStatus.Finished())
    #resume(self)

  def position_reached_callback(self, req):
    print("[DRIVE TO] position_reached_callback")
    self.executionFinished(TstML.Executor.ExecutionStatus.Finished())
    #resume(self)

  def start(self):
    if(self.node().hasParameter(TstML.TSTNode.ParameterType.Specific, "use-motion-planner")):
        use_motion_planner = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "use-motion-planner")
    else:
        use_motion_planner = False
    #x = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "x")
    point = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "p")

    max_veclocity = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "maximum-speed")

    heading = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "heading")

    print("use motion planner - "+str(use_motion_planner))
    print(point)
    print("Max Veclocityc - "+str(max_veclocity))
    print("Heading - "+str(heading))

    if(max_veclocity == None):
        max_veclocity = 0.5

    if(heading == None):
        heading = 0

    max_veclocity_message = geometry_msgs.msg.Twist()
    max_veclocity_message.linear.x = max_veclocity
    max_veclocity_message.linear.y = max_veclocity
    max_veclocity_message.linear.z = max_veclocity

    max_veclocity_message.angular.x = (3*max_veclocity)
    max_veclocity_message.angular.y = (3*max_veclocity)
    max_veclocity_message.angular.z = (3*max_veclocity)

    self.max_velocity_pub.publish(max_veclocity_message)
    print("[DRIVE TO]max_velocity_pub publish")

    dest_point = geometry_msgs.msg.PoseStamped()
    dest_point.pose.position.x = point['x']
    dest_point.pose.position.y = point['y']
    dest_point.pose.position.z = point['z']

    quat = tf.transformations.quaternion_from_euler(0, 0, heading)
    dest_point.pose.orientation.x = quat[0]
    dest_point.pose.orientation.y = quat[1]
    dest_point.pose.orientation.z = quat[2]
    dest_point.pose.orientation.w = quat[3]

    if(use_motion_planner):
        self.to_waypointControl_pub.publish()
        time.sleep(1)

        #waypoint_message = nav_msgs.msg.Path()
        #waypoint_message.poses.append(dest_point)
        #self.cmd_waypoint_pub.publish(waypoint_message)
        self.to_destination_pub.publish(dest_point)
        print("[DRIVE TO] to_destination_pub publish")
    else:
        self.to_positionControl_pub.publish()
        time.sleep(1)
        
        self.cmd_position_pub.publish(dest_point)
        print("[DRIVE TO] cmd_position_pub publish")


    return TstML.Executor.ExecutionStatus.Started()
  def pause(self):
    return TstML.Executor.ExecutionStatus.Paused()
  def resume(self):
    return TstML.Executor.ExecutionStatus.Running()
  def stop(self):
    return TstML.Executor.ExecutionStatus.Finished()
  def abort(self):
    return TstML.Executor.ExecutionStatus.Aborted()

# Associate the EchoExecutor to the "echo" model
