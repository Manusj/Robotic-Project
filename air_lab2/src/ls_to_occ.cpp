#include "air_lab2/occ.h"
#include <ros/node_handle.h>
#include <ros/service.h>
#include <ros/subscriber.h>
#include <atomic>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

class LStoOCC {

public:
  ros::Subscriber m_lsSub;
  ros::ServiceServer  m_mapRequest;
  tf::TransformListener m_tfListener;

  void laserScanCallback(const sensor_msgs::LaserScanPtr& _message)
  {
    // In one of your callback or elsewhere:
    // We want to know the transformation from source_frame_id to
    // destination_frame_id at time time_stamp
    tf::StampedTransform transform;
    try
    {
      // Define a 1s timeout
      ros::Duration timeout(1.0);
      // First, lets wait a bit to make synchronize our TF tree and
      // make sure we have received the transform
      m_tfListener.waitForTransform("odom",_message->header.frame_id, _message->header.stamp, timeout);
      // Then lets get the transformation
      m_tfListener.lookupTransform("odom",_message->header.frame_id, _message->header.stamp, transform);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR_STREAM( "Failed to get the transformation: "
      << ex.what() << ", quitting callback");
      return;
    }
    float alpha_i = _message->angle_min - _message->angle_increment;
    tf::Vector3 temp{cos(alpha_i),sin(alpha_i),0};
    tf::Vector3 orgin{0,0,0};
    tf::Vector3 direction = (transform * temp - transform*orgin).normalize();

    m_occ->ensureInitialise(transform);
    //return;
    for(float range : _message->ranges)
    {

      if(std::isnan(range))
        m_occ->rayTrace(m_robot_size,transform*orgin,direction,_message->range_max,false);
      else
        m_occ->rayTrace(m_robot_size,transform*orgin,direction,range,true);
      alpha_i = alpha_i + _message->angle_increment;
      temp = tf::Vector3(cos(alpha_i),sin(alpha_i),0);
      direction = (transform * temp - transform*orgin).normalize();
    }

  }

  bool mapService(nav_msgs::GetMapRequest& _req, nav_msgs::GetMapResponse& _resp)
  {
    return m_occ->requestMap(_resp);
  }

  LStoOCC(const ros::NodeHandle& _nodeHandle)
  : m_nodeHandle(_nodeHandle), m_occ(nullptr),
  m_cell_size(0.1), m_robot_size(1.0)
  {
    m_lsSub = m_nodeHandle.subscribe("scan", 1, &LStoOCC::laserScanCallback, this);
    m_mapRequest = m_nodeHandle.advertiseService("map_request", &LStoOCC::mapService, this);
    double grid_cell_size = 0.1;
    ros::NodeHandle private_nodehandle("~");

    //not sure about the m_robot_size might be another...
    private_nodehandle.getParam("grid_cell_size", m_cell_size);
    private_nodehandle.getParam("robot_size", m_robot_size);
    std::cout<<"grid_cell_size"<<m_cell_size<<std::endl;
    std::cout<<"m_robot_size"<<m_robot_size<<std::endl;
    m_occ = new OCC(m_cell_size, 2*m_robot_size);

  }


private:
  ros::NodeHandle m_nodeHandle;
  OCC* m_occ;
  double m_cell_size, m_robot_size;
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ls_to_occ");
  ros::NodeHandle n;
  LStoOCC ptd(n);
  ros::spin();
  return 0;
}
