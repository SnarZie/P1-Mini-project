#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Char.h"
#include <iostream>
#include <stdlib.h>
#include <sstream>

ros::Subscriber chatter_sub;
ros::Subscriber chatter2_sub;

void _listen_cb(const std_msgs::Char::ConstPtr& msg)
{
	ROS_INFO("I Heard : [%c] on chatter", msg->data);
	switch(msg->data) {
		case 'A' :
			ROS_INFO("Please put some coffee on me!");
		break;
		case 'B' :
			ROS_INFO("Please put a pencil on me!");
		break;
		case 'C' :
			ROS_INFO("Please put the box on me!");
		break;
		case 'D' :
			ROS_INFO("Please put a cup on me!");
		break;
		case 'E' :
			ROS_INFO("Please put the Ipad on me!");
		break;
		case 'F' :
			ROS_INFO("Please put some idk on me!");
		break;
	}
}

void _clear_cb(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard : [%s] on chatter 2", msg->data.c_str());
	if (msg->data == "clear")
		system("clear");
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "phantom");
	
	ros::NodeHandle n;
        
		chatter_sub = n.subscribe( "chatter", 10, &_listen_cb);
		chatter2_sub = n.subscribe( "chatter2", 10, &_clear_cb);

		

	ros::spin();
	return 0;
}