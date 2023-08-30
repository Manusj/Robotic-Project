#!/usr/bin/env python3

import rospy
import tf

import nav_msgs.msg
import geometry_msgs.msg
import nav_msgs.srv
import std_msgs.msg

class move_to_point:
    def __init__(self):
        self.destinationSub = rospy.Subscriber('destination', geometry_msgs.msg.PoseStamped, self.destination_callback)
        self.outputPathPub = rospy.Publisher('planned_path', nav_msgs.msg.Path, queue_size = 1)
        self.motionPlanService = rospy.ServiceProxy('plan_path', nav_msgs.srv.GetPlan)
        self.tf_listener = tf.TransformListener()
        self.robot_frame = rospy.get_param('~robot_frame')
        self.skip_first_point = rospy.get_param('~skip_first_point', True)
        self.noPathPub = rospy.Publisher('/husky0/noplan', std_msgs.msg.Empty, queue_size = 1)

    def destination_callback(self, destination):
        print("destination:")
        print(destination)
        try:
              (trans,rot) = self.tf_listener.lookupTransform('odom', self.robot_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to get transformation from 'odom' to 'base_footprint'!")
            return
        print(trans)
        try:
            print("before")
            plan = self.motionPlanService(geometry_msgs.msg.PoseStamped(destination.header, geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*trans), geometry_msgs.msg.Quaternion(*rot))) , destination, 0.1)
            print("after")
            print(plan)
            plan_path = plan.plan

            if len(plan_path.poses) == 0:
                print("plan length = 0")
                self.noPathPub.publish(std_msgs.msg.Empty())
                return

            if self.skip_first_point:
              plan_path.poses.pop(0)

            self.outputPathPub.publish(plan.plan)

            rospy.loginfo("\o/")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
            return

if __name__ == '__main__':
    rospy.init_node('move_to_point', anonymous=False)
    ec = move_to_point()
    rospy.spin()
