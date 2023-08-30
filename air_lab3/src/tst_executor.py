#!/usr/bin/env python3

import math
import echo_executor
import drive_to_executor
import explore_executor
import record_executor
# rosoy is the main API for ROS
import rospy
import air_lab3.srv
import TstML
import rospkg
import TstML.Executor
import std_srvs.srv


class tst_executor:
    tst_registry = TstML.TSTNodeModelsRegistry()
    tst_executor_registry = TstML.Executor.NodeExecutorRegistry()

    def __init__(self):
        service = rospy.Service('execute_tst', air_lab3.srv.ExecuteTst, self.callback)

        abort_service = rospy.Service('abort', std_srvs.srv.Empty, self.abort_callback)
        stop_service = rospy.Service('stop', std_srvs.srv.Empty, self.stop_callback)
        pause_service = rospy.Service('pause', std_srvs.srv.Empty, self.pause_callback)
        resume_service = rospy.Service('resume', std_srvs.srv.Empty, self.resume_callback)

        self.tst_registry.loadDirectory(rospkg.RosPack().get_path("air_tst") + "/configs")

        self.tst_executor_registry.registerNodeExecutor(
            self.tst_registry.model("seq"),
            TstML.Executor.DefaultNodeExecutor.Sequence)

        self.tst_executor_registry.registerNodeExecutor(
            self.tst_registry.model("conc"),
            TstML.Executor.DefaultNodeExecutor.Concurrent)

        self.tst_executor_registry.registerNodeExecutor(
             self.tst_registry.model("echo"), echo_executor.EchoExecutor)

        self.tst_executor_registry.registerNodeExecutor(
              self.tst_registry.model("drive-to"), drive_to_executor.DriveToExecutor)

        self.tst_executor_registry.registerNodeExecutor(
              self.tst_registry.model("explore"), explore_executor.ExploreExecutor)

        self.tst_executor_registry.registerNodeExecutor(
              self.tst_registry.model("record"), record_executor.RecordExecutor)

    def abort_callback(self, req):
        return TstML.Executor.Executor.abort()

    def stop_callback(self, req):
        return TstML.Executor.Executor.stop()

    def pause_callback(self, req):
        return TstML.Executor.Executor.pause()

    def resume_callback(self, req):
        return TstML.Executor.Executor.resume()

    def callback(self, req):
        tst_node = TstML.TSTNode.load(req.tst_file, self.tst_registry)
        tst_executor = TstML.Executor.Executor(tst_node, self.tst_executor_registry)
        tst_executor.start()
        status = tst_executor.waitForFinished()
        message = ""
        if status.type() == TstML.Executor.ExecutionStatus.Type.Finished:
            message = "Execution successful"
            print("Execution successful")
        else:
            message = "Execution failed: {}".format(status.message())
            print("Execution failed: {}".format(status.message()))
        return (not status.isFailed()), message


if __name__ == '__main__':
    tst_executor = tst_executor()
    # Initialise the ROS sub system
    rospy.init_node('tst_executor', anonymous=False)
    # Create an instance of our controller
    rospy.spin()
