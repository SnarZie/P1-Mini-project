#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <stdlib.h>
#include <sstream>

//This program puplishes what the user writes onto the chatter topic
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "talker");
 
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<std_msgs::Char>("chatter", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::Char msg; //Message object, that can contain data
    
    std::cout << "There are six points\n to go to A: A\n to go to B: B\n to go to C: C\n to go to D: D\n to go to E: E\n to go to F: F\n\n      User input:    ";
    std::cin >> msg.data; //Add user input to the data of msg

    ROS_INFO("%c", msg.data);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
