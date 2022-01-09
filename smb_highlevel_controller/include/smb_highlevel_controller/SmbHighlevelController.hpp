#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>

namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber;
	ros::Publisher cmd_publisher;
	ros::Publisher vis_publisher;
	ros::ServiceServer service;
	void scanCallback(const sensor_msgs::LaserScan& scan);
	bool stopCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);
	std::string topic_name;
	int queue_size;
	float p_gain;
	bool stop;
};

} /* namespace */
