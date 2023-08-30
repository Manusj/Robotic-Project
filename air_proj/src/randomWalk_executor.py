#!/usr/bin/env python3
import rospy
import air_lab3.srv
import TstML
import rospkg
import TstML.Executor

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

# world 1 - 40, -44

class RandomWalkExecutor(TstML.Executor.AbstractNodeExecutor):
  def __init__(self, node, context):
    super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
          context)

    self.to_waypointControl_pub = rospy.Publisher("/husky0/to_waypoint_control",
              std_msgs.msg.Empty, queue_size = 1)
    self.to_destination_pub = rospy.Publisher("/husky0/destination",
              geometry_msgs.msg.PoseStamped, queue_size = 1)

    self.max_velocity_pub = rospy.Publisher("/husky0/max_velocity",
              geometry_msgs.msg.Twist, queue_size = 1)
    self.cmd_waypoint_pub = rospy.Publisher("/husky0/cmd_waypoints",
              nav_msgs.msg.Path, queue_size = 1)

    self.waypoint_finished_sub =  rospy.Subscriber("/husky0/waypoints_finished",
              std_msgs.msg.Empty, self.waypoints_finished_callback)

    self.display_marker_pub = rospy.Publisher('/husky0/semantic_sensor_visualisation', visualization_msgs.msg.MarkerArray, queue_size=10, latch = True)

    self.queryServiceClient = rospy.ServiceProxy('/husky0/kDBServerNodelet/sparql_query', lrs_kdb_msgs.srv.QueryDatabase)

    time.sleep(2.0)

    self.trippleServiceCall = rospy.ServiceProxy('/husky0/kDBServerNodelet/insert_triples', lrs_kdb_msgs.srv.InsertTriples)

    self.graphname = "semanticobject"
    self.topics = "/husky0/semantic_sensor"

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

  def semantic_topic_received(self, msg):
    try:
        self.tf_listener.waitForTransform('odom', msg.point.header.frame_id, msg.point.header.stamp, rospy.Duration(1.0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Failed to get transformation from 'odom' to '"+str(msg.point.header.frame_id) +"'!")

    point_stamped = msg.point
    point_in_world = self.tf_listener.transformPoint('odom', point_stamped)

    kclass = msg.klass
    uuid = msg.uuid
    x = point_in_world.point.x
    y = point_in_world.point.y

    print("class - "+str(kclass))
    print("uuid - "+str(uuid))
    print("x - "+str(x))
    print("y - "+str(y))

    querycommand = lrs_kdb_msgs.srv.QueryDatabaseRequest()

    querycommand.graphnames.append(self.graphname)
    querycommand.format = "json"
    querycommand.query = """PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>
PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>
SELECT ?x ?y WHERE { <"""+str(uuid) + "> a <" +str(kclass)+"""> ;
properties:location [ gis:x ?x; gis:y ?y ] . }"""
    result = self.queryServiceClient(querycommand)
    data = json.loads(result.result)

    if(len(data["results"]["bindings"])==0):
        writecommand = lrs_kdb_msgs.srv.InsertTriplesRequest()
        writecommand.graphname=self.graphname
        writecommand.format = "ttl"

        writecommand.content = """@prefix gis: <http://www.ida.liu.se/~TDDE05/gis>
@prefix properties: <http://www.ida.liu.se/~TDDE05/properties>
<"""+uuid+"""> a <"""+ kclass+""">;
properties:location [ gis:x """ + str(x) + """; gis:y """ + str(y) + """ ] ."""

        print("************insert query**************")
        print(writecommand.content)

        result = self.trippleServiceCall(writecommand)


        marker = visualization_msgs.msg.Marker()
        marker.id     = 1242 # identifier the marker, should be unique
        marker.header.frame_id = "odom"
        marker.type   = visualization_msgs.msg.Marker.CUBE_LIST
        marker.action = 0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0

        querycommand = lrs_kdb_msgs.srv.QueryDatabaseRequest()
        querycommand.graphnames.append("semanticobject")
        querycommand.format = "json"
        querycommand.query = """PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>
    PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>
    SELECT ?x ?y ?uid ?klass WHERE { ?uid a ?klass ;
    properties:location [ gis:x ?x; gis:y ?y ] . }"""

        result = self.queryServiceClient(querycommand)

        data = json.loads(result.result)
        for row in data["results"]["bindings"]:
            x = row["x"]["value"]
            y = row["y"]["value"]
            marker.points.append(geometry_msgs.msg.Point(float(x), float(y), 0))
            if(row["klass"]["value"] == "table"):
                marker.colors.append(std_msgs.msg.ColorRGBA(1.0, 0.5, 0.5, 1.0))
            elif(row["klass"]["value"] == "human"):
                marker.colors.append(std_msgs.msg.ColorRGBA(0.5, 1.0, 0.5, 1.0))

        self.display_marker_pub.publish(visualization_msgs.msg.MarkerArray([marker]))


  def start(self):
    print("Random Walk Start called")
    max_veclocity = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "maximum-speed")

    self.topic_sub =  rospy.Subscriber(self.topics, SemanticObservation, self.semantic_topic_received)

    self.tf_listener = tf.TransformListener()

    if(max_veclocity == None):
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



    return TstML.Executor.ExecutionStatus.Started()
  def pause(self):
    return TstML.Executor.ExecutionStatus.Paused()
  def resume(self):
    return TstML.Executor.ExecutionStatus.Running()
  def stop(self):
    return TstML.Executor.ExecutionStatus.Finished()
  def abort(self):
    return TstML.Executor.ExecutionStatus.Aborted()
