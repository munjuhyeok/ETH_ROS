#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

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
	void scanCallback(const sensor_msgs::LaserScan& scan);
	std::string topic_name;
	int queue_size;
	float p_gain;
};

} /* namespace */
