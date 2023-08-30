#include "air_lab2/motion_planner_interface.h"
#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/subscriber.h>
#include <atomic>
#include <nav_msgs/GetPlan.h>
//#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class Motion_planner {

public:
  ros::ServiceServer  m_planService;

  bool planPath(nav_msgs::GetPlanRequest& _req, nav_msgs::GetPlanResponse& _resp)
  {
  printf("planPath is called");
    geometry_msgs::PoseStamped pos_start = _req.start;
    geometry_msgs::PoseStamped pos_goal = _req.goal;
    std::string frame_id;
    std::vector<std::pair<double, double>> path;
    std::vector<geometry_msgs::PoseStamped> poses;
    path = m_motion_planner_interface->planPath<nav_msgs::GetMap>(pos_start.pose.position.x,pos_start.pose.position.y, pos_goal.pose.position.x, pos_goal.pose.position.y, &frame_id);
    geometry_msgs::PoseStamped position;
    for(auto p = path.begin(); p != path.end(); p++)
    {

        position.pose.position.x = p->first;
        position.pose.position.y = p->second;
        auto prev_element = p-1;
        auto next_element = p+1;
        if(prev_element >= path.begin() and next_element < path.end())
        {
          auto y = next_element->second - prev_element->second;
          auto x = next_element->first - prev_element->first;
          tf::Quaternion orientation;
          orientation.setRPY( 0, 0, atan(y/x));

          position.pose.orientation.x = orientation.getX();
          position.pose.orientation.y = orientation.getY();
        }
        else if(prev_element < path.begin())
        {
          tf::Quaternion orientation;
          orientation.setRPY( 0, 0, atan(next_element->second/next_element->first));

          position.pose.orientation.x = orientation.getX();
          position.pose.orientation.y = orientation.getY();
        }
        else
        {
          tf::Quaternion orientation;
          orientation.setRPY( 0, 0, atan(prev_element->second/prev_element->first));

          position.pose.orientation.x = orientation.getX();
          position.pose.orientation.y = orientation.getY();
        }
        poses.push_back(position);
    }
  //  _resp.plan.poses_length = poses.size();
  //_resp.plan.header.frame_id = "odom";
  //_resp.plan.header.stamp    = ros::Time::now();
    _resp.plan.poses = poses;

    return true;
  }

  Motion_planner(const ros::NodeHandle& _nodeHandle)
  : m_nodeHandle(_nodeHandle), m_motion_planner_interface(new MotionPlannerInterface),
  m_cell_size(0.1), m_robot_size(1.0)
  {
      m_planService = m_nodeHandle.advertiseService("plan_path", &Motion_planner::planPath, this);
      printf("test...");
  }



private:
  ros::NodeHandle m_nodeHandle;
  MotionPlannerInterface *m_motion_planner_interface;
  double m_cell_size, m_robot_size;
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle n;
  Motion_planner ptd(n);
  ros::spin();
  return 0;
}
