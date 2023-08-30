import rospy
import visualization_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import lrs_kdb_msgs.srv
import tf
import time
from air_simple_sim_msgs.msg import SemanticObservation
import json
import nav_msgs

class visualize_semantic_objects:
    def __init__(self):
        self.graphname = "semanticobject"
        self.topics = "/husky0/semantic_sensor"

        self.queryServiceClient = rospy.ServiceProxy('/husky0/kDBServerNodelet/sparql_query', lrs_kdb_msgs.srv.QueryDatabase)
        time.sleep(2.0)
        self.trippleServiceCall = rospy.ServiceProxy('/husky0/kDBServerNodelet/insert_triples', lrs_kdb_msgs.srv.InsertTriples)
        self.display_marker_pub = rospy.Publisher('/husky0/semantic_sensor_visualisation', visualization_msgs.msg.MarkerArray, queue_size=10, latch = True)

        self.semantic_topic_sub =  rospy.Subscriber(self.topics, SemanticObservation, self.semantic_topic_received)
        self.tf_listener = tf.TransformListener()

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

        #print("class - "+str(kclass))
        #print("uuid - "+str(uuid))
        #print("x - "+str(x))
        #print("y - "+str(y))

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


class visualize_frontier:
    def __init__(self):
        self.OccupancyGrid_sub =  rospy.Subscriber("/husky0/map_display", nav_msgs.msg.OccupancyGrid, self.OccupancyGrid_received)
        self.frontier_pub = rospy.Publisher('/husky0/frontier', nav_msgs.msg.GridCells, queue_size=10, latch = True)
        self.once = True


    def OccupancyGrid_received(self, occupancyGrid):
        origin_x = occupancyGrid.info.origin.position.x
        origin_y = occupancyGrid.info.origin.position.y
        resolution = occupancyGrid.info.resolution
        occupancy_data = occupancyGrid.data
        unknown_data_index = [i for i, d in enumerate(occupancy_data) if d == -1]
        frontier = geometry_msgs.msg.Polygon()

        for i in range(0,occupancyGrid.info.width):
            for j in range(0,occupancyGrid.info.height):
                if(occupancy_data[i*occupancyGrid.info.width+j]!=-1 and occupancy_data[i*occupancyGrid.info.width+j]!=100):
                    neighbours=[(i-1,j-1),(i-1,j),(i-1,j+1),(i,j-1),(i,j+1),(i+1,j-1),(i+1,j),(i+1,j+1)]
                    for k in neighbours:
                        if(occupancy_data[k[0]*occupancyGrid.info.width+k[1]]==-1):
                            point = geometry_msgs.msg.Point32()
                            point.x = j*resolution+origin_x
                            point.y = i*resolution+origin_y
                            point.z = 0
                            frontier.points.append(point)
                            break

        frontier_cell_msg = nav_msgs.msg.GridCells()
        frontier_cell_msg.header.frame_id = "odom"
        frontier_cell_msg.cell_width = resolution
        frontier_cell_msg.cell_height = resolution
        frontier_cell_msg.cells = frontier.points


        if(self.once):
            self.frontier_pub.publish(frontier_cell_msg)
            self.once = True

class visualize_bountry:
    def __init__(self, width):
        print("visualize_bountry")
        self.boundary_pub = rospy.Publisher('/husky0/boundary', geometry_msgs.msg.PolygonStamped, queue_size=10, latch = True)

        frontier = geometry_msgs.msg.Polygon()
        point1 = geometry_msgs.msg.Point32()
        point1.x = -width
        point1.y = -width
        point1.z = 0
        point2 = geometry_msgs.msg.Point32()
        point2.x = -width
        point2.y = width
        point2.z = 0
        point3 = geometry_msgs.msg.Point32()
        point3.x = width
        point3.y = width
        point3.z = 0
        point4 = geometry_msgs.msg.Point32()
        point4.x = width
        point4.y = -width
        point4.z = 0
        point5 = geometry_msgs.msg.Point32()
        point5.x = -width
        point5.y = -width
        point5.z = 0
        frontier.points.append(point1)
        frontier.points.append(point2)
        frontier.points.append(point3)
        frontier.points.append(point4)
        frontier.points.append(point5)
        frontier_cell_msg = geometry_msgs.msg.PolygonStamped()
        frontier_cell_msg.header.frame_id = "odom"
        frontier_cell_msg.polygon = frontier
        self.boundary_pub.publish(frontier_cell_msg)
