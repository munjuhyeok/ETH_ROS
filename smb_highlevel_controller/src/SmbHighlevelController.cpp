#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  if(!nodeHandle_.getParam("topic_name", topic_name)){
    ROS_ERROR("Could not find topic_name parameter!");
  }
  if(!nodeHandle_.getParam("queue_size", queue_size)){
    ROS_ERROR("Could not find queue_size parameter!");
  }
  if(!nodeHandle_.getParam("p_gain", p_gain)){
    ROS_ERROR("Could not find p_gain parameter!");
  }
  stop=true;

  subscriber = nodeHandle_.subscribe(topic_name, queue_size, &SmbHighlevelController::scanCallback, this);
  cmd_publisher = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  vis_publisher = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker",1);
  service = nodeHandle_.advertiseService("/stop_smb", &SmbHighlevelController::stopCallback, this);
}

SmbHighlevelController::~SmbHighlevelController()
{
}

void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan& scan){
  float min=scan.range_max;
  int min_index;
  for (int i = 0; i < scan.ranges.size(); i++){
    float range = scan.ranges.at(i);
    if (min > range){
      min = range;
      min_index = i;
    }
  }
  ROS_INFO("smallest distance measurement : %f", min);

  float angle_diff = scan.angle_min + scan.angle_increment * min_index;
  geometry_msgs::Twist cmd_vel;
  if(stop){
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }else{
    cmd_vel.linear.x = 1.0;
    cmd_vel.angular.z = p_gain * angle_diff;
  }
  cmd_publisher.publish(cmd_vel);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = min * cos(angle_diff);
  marker.pose.position.y = min * sin(angle_diff);
  marker.pose.position.z = 0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  vis_publisher.publish(marker);
}

bool SmbHighlevelController::stopCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response){
  if(request.data == true){
    stop = true;
    response.message = "smb stopped";
    ROS_INFO("smb stopped");
  }else{
    stop = false;
    response.message = "smb started";
    ROS_INFO("smb started");
  }
}


} /* namespace */
