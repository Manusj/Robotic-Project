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
import psutil
import signal
import subprocess
import json
import lrs_kdb_msgs.srv
from air_simple_sim_msgs.msg import SemanticObservation


class RecordSemanticsExecutor(TstML.Executor.AbstractNodeExecutor):
  def __init__(self, node, context):
    super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
          context)
    self.queryServiceClient = rospy.ServiceProxy('/husky0/kDBServerNodelet/sparql_query', lrs_kdb_msgs.srv.QueryDatabase)

    time.sleep(2.0)

    self.trippleServiceCall = rospy.ServiceProxy('/husky0/kDBServerNodelet/insert_triples', lrs_kdb_msgs.srv.InsertTriples)

    time.sleep(2.0)

  def start(self):
    self.graphname = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "graphname")
    self.topics = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "topic")

    self.topic_sub =  rospy.Subscriber(self.topics, SemanticObservation, self.topic_received)

    self.tf_listener = tf.TransformListener()

    return TstML.Executor.ExecutionStatus.Started()

  def topic_received(self, msg):
    print(msg)
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

        result = self.trippleServiceCall(writecommand)
        print("************insert query**************")
        print(writecommand.content)

  def pause(self):
    return TstML.Executor.ExecutionStatus.Paused()
  def resume(self):
    return TstML.Executor.ExecutionStatus.Running()
  def stop(self):
    return TstML.Executor.ExecutionStatus.Finished()
  def abort(self):
    return TstML.Executor.ExecutionStatus.Aborted()
