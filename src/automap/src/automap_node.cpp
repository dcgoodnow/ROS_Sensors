#include "ros/ros.h"
#include "automove.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "automap_node");
   ros::NodeHandle nh;

   automove am;
   am.startMoving();
}
