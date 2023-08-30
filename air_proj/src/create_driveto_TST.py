#!/usr/bin/env python3

import rospy
import visualization_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import lrs_kdb_msgs.srv
import json
import air_lab3.srv

queryServiceClient = rospy.ServiceProxy('/husky0/kDBServerNodelet/sparql_query', lrs_kdb_msgs.srv.QueryDatabase)

queryTstExcuteClient = rospy.ServiceProxy('/husky0/execute_tst', air_lab3.srv.ExecuteTst)

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

tst_data = {}
tst_data["children"] = list()
tst_data["common_params"] = {}
tst_data["name"] =  "seq"
tst_data["params"] = {}

for row in data["results"]["bindings"]:
    if(row["klass"]["value"] == "human"):
        x = row["x"]["value"]
        y = row["y"]["value"]
        tst_data["children"].append({"children": [], "common_params" : {},  "name": "drive-to",
                                    "params": {"p": { "rostype": "Point","x": float(x),"y": float(y),"z": 0},"use-motion-planner": False}})

print(tst_data)
with open('/home/mansj125/TDDE05/catkin_ws/src/air_labs/air_lab4/src/data.json','w') as outfile:
    json.dump(tst_data, outfile, indent=4)

tst_exceute_command = air_lab3.srv.ExecuteTstRequest()
tst_exceute_command.tst_file = '/home/mansj125/TDDE05/catkin_ws/src/air_labs/air_lab4/src/data.json'
result = queryTstExcuteClient(tst_exceute_command)
print(result)
