#!/usr/bin/env python3

import rospy
import visualization_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import lrs_kdb_msgs.srv
import json

class visualise_semantic_objects:
    pass


if __name__ == '__main__':
    rospy.init_node('visualise_semantic_objects', anonymous=False)

    display_marker_pub = rospy.Publisher('/husky0/semantic_sensor_visualisation', visualization_msgs.msg.MarkerArray, queue_size=10, latch = True)

    queryServiceClient = rospy.ServiceProxy('/husky0/kDBServerNodelet/sparql_query', lrs_kdb_msgs.srv.QueryDatabase)

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

    print(querycommand.query)
    result = queryServiceClient(querycommand)
    print("********RESULT***********")
    print(result.result)

    data = json.loads(result.result)
    '''
    marker.points.append(geometry_msgs.msg.Point(2, 3, 0))
    marker.points.append(geometry_msgs.msg.Point(5, 2, 0))
    marker.colors.append(std_msgs.msg.ColorRGBA(1.0, 0.5, 0.5, 1.0))
    marker.colors.append(std_msgs.msg.ColorRGBA(0.5, 1.0, 0.5, 1.0))
    '''
    for row in data["results"]["bindings"]:
        x = row["x"]["value"]
        y = row["y"]["value"]
        print("X - "+str(x))
        print("Y - "+str(y))
        print("class - "+str(row["klass"]["value"]))
        marker.points.append(geometry_msgs.msg.Point(float(x), float(y), 0))
        if(row["klass"]["value"] == "table"):
            marker.colors.append(std_msgs.msg.ColorRGBA(1.0, 0.5, 0.5, 1.0))
            print("Class - Table")
        elif(row["klass"]["value"] == "human"):
            marker.colors.append(std_msgs.msg.ColorRGBA(0.5, 1.0, 0.5, 1.0))
            print("Class - Human")

    print(marker.points)
    print(marker.colors)

    display_marker_pub.publish(visualization_msgs.msg.MarkerArray([marker]))

    rospy.spin()
