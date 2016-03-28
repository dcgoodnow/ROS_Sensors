#include "automove.h"
#include "geometry_msgs/Twist.h"

automove::automove()
{
	keepMoving = true;

	// Advertise a new publisher for the simulated robot's velocity command topic
	commandPub = node.advertise<geometry_msgs::Twist>("create/cmd_vel", 10);

	// Subscribe to the simulated robot's laser scan topic
	laserSub = node.subscribe("create/scan", 1, &automove::scanCallback, this);
}

// Send a velocity command
void automove::moveForward() {
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = FORWARD_SPEED_MPS;
	commandPub.publish(msg);
};

void automove::stopMoving(){
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = 0;
	commandPub.publish(msg);
};

void automove::newAngle() {
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = BACKWARD_SPEED_MPS;
	msg.angular.z = 5;
	commandPub.publish(msg);
};

// Process the incoming laser scan message
void automove::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Find the closest range between the defined minimum and maximum angles
	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < closestRange) {
			closestRange = scan->ranges[currIndex];
		}
	}

	ROS_INFO_STREAM("Closest range: " << closestRange);

	if (closestRange < MIN_PROXIMITY_RANGE_M) {
		ROS_INFO("Collision Warning!");
		stopMoving();
		newAngle();
	}
}

void automove::startMoving()
{
	ros::Rate rate(10);
	ROS_INFO("Start moving");

	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while (ros::ok() && keepMoving) {
		moveForward();
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep();
	}
}
