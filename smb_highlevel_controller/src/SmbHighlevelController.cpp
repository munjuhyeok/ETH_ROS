#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  std::string topic_name;
  int queue_size;
  if(!nodeHandle_.getParam("topic_name", topic_name)){
    ROS_ERROR("Could not find topic_name parameter!");
  }
  if(!nodeHandle_.getParam("queue_size", queue_size)){
    ROS_ERROR("Could not find topic_name parameter!");
  }
  subscriber = nodeHandle_.subscribe(topic_name, queue_size, &SmbHighlevelController::scanCallback, this);
}

SmbHighlevelController::~SmbHighlevelController()
{
}

void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan& scan){
  float min=scan.range_max;
  for (float range : scan.ranges){
    min = min > range? range:min;
  }
  ROS_INFO("smallest distance measurement : %f", min);
}

} /* namespace */
