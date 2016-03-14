#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class automove
{
public:
	// Tunable parameters
	constexpr static double FORWARD_SPEED_MPS = 0.5;
	constexpr static double BACKWARD_SPEED_MPS = -0.5;
	constexpr static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
	constexpr static double MAX_SCAN_ANGLE_RAD = 30.0/180*M_PI;
	constexpr static float MIN_PROXIMITY_RANGE_M = 0.5; // Should be smaller than sensor_msgs::LaserScan::range_max
	constexpr static double ANGULAR_SPEED = 0.2;
	automove();
	void startMoving();
	void stopMoving();

private:
	ros::NodeHandle node;
	ros::Publisher commandPub; // Publisher to the robot's velocity command topic
	ros::Subscriber laserSub; // Subscriber to the robot's laser scan topic
	bool keepMoving; // Indicates whether the robot should continue moving

	void moveForward();
	void newAngle();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};
