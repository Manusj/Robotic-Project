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

class RecordExecutor(TstML.Executor.AbstractNodeExecutor):
  def __init__(self, node, context):
    super(TstML.Executor.AbstractNodeExecutor, self).__init__(node,
          context)
    time.sleep(2.0)

  def terminate_process_and_children(self, p):
    """ Take a process as argument, and kill all the children
    recursively.
    """
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait() # we wait for children to terminate
    self.executionFinished(TstML.Executor.ExecutionStatus.Finished())

  def start(self):
    filename = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "filename")
    topics = self.node().getParameter(TstML.TSTNode.ParameterType.Specific, "topics")


    print("filename - "+str(filename))
    print("topics - "+str(topics))

    self.p = subprocess.Popen(["rosbag", "record", "-O" "/home/mansj125/TDDE05/catkin_ws/src/air_labs/air_lab3/"+filename, topics])
    print(type(self.p))


    return TstML.Executor.ExecutionStatus.Started()
  def pause(self):
    return TstML.Executor.ExecutionStatus.Paused()
  def resume(self):
    return TstML.Executor.ExecutionStatus.Running()
  def stop(self):
    self.terminate_process_and_children(self.p)
    return TstML.Executor.ExecutionStatus.Finished()
  def abort(self):
    return TstML.Executor.ExecutionStatus.Aborted()
