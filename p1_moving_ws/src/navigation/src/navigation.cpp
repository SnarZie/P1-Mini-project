#include <algorithm>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/String.h"
#include "std_msgs/Char.h"
#include <iostream>


class Route
{
private:
    unsigned int points_initialized;
    geometry_msgs::PointStamped zero, pointA, pointB, pointC, pointD, pointE, pointF;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client;
    visualization_msgs::MarkerArray marker_array;
    ros::Publisher marker_pub;
    ros::Subscriber click_sub;
    ros::Subscriber chatter_sub;
    ros::Publisher chatter2_pub;

    //A method that sends a point to the robot
    void _send_goal(const geometry_msgs::PointStamped& goal_point, const geometry_msgs::PointStamped& goal_point2)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = goal_point.header.frame_id;
        goal.target_pose.pose.position = goal_point.point;
        goal.target_pose.pose.orientation.z = 1;
        client.sendGoal(goal, boost::bind(&Route::_send2_goal, this , goal_point2)); 
        //When the goal is reached it opens the "_target_reached_cb" function
    }

    void _send2_goal(const geometry_msgs::PointStamped& goal_point)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = goal_point.header.frame_id;
        goal.target_pose.pose.position = goal_point.point;
        goal.target_pose.pose.orientation.z = 1;
        client.sendGoal(goal, boost::bind(&Route::_target_reached_cb, this, _1, _2)); 
        //When the goal is reached it opens the "_target_reached_cb" function
    }


    //A method that is run when the robot reaches it's goal point
    void _target_reached_cb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
    {
        //Make the robot go back to zero 
        system("pause");
        _send_goal(zero);
        std_msgs::String msg;
        msg.data = "clear";
        chatter2_pub.publish(msg);
    }

    //A method that takes arguments to create a visual marker
    void _create_marker(double red, double green, double blue, double alpha, bool down, int id, geometry_msgs::PointStamped marker_name)
    {
        visualization_msgs::Marker marker; //Create marker object, called marker
        marker.header.stamp = ros::Time::now();
        marker.ns = "bus_stops";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 1.0;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
        marker.color.a = alpha;
        
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0.7071;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = -0.7071;
        
        marker.lifetime = ros::Duration();

        marker.header.frame_id = marker_name.header.frame_id;
        marker.id = id;
        
        marker.pose.position = marker_name.point;

        //Make the opposite go downward (opposite of standard setting)
        if(down)
        {
            marker.pose.position.z += marker.scale.x; //Moves arrow above ground
            marker.pose.orientation.w = 0.7071;
        }

        marker_array.markers.push_back(marker); //Push the marker object to the array of markers
        marker_pub.publish(marker_array); //Publish the marker_array
    }

    //The puplished points from the rviz are used to create points in the program that are then put into the "_create_marker" function
    void _clicked_point_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
    {

        switch(points_initialized) {
            case 0:
                ROS_INFO("Placed the station");
                zero = *msg;
                _create_marker(1, 1, 1, 1, true, points_initialized, zero);
            break;  
            case 1:
                pointA = *msg;
                ROS_INFO("Changed the point A");
                _create_marker(1, 0, 0, 1, false, points_initialized, pointA); 
            break;
            case 2:
                pointB = *msg;
                ROS_INFO("Changed the point B");
                _create_marker(0, 1, 0, 1, false, points_initialized, pointB);
            break;
            case 3:
                pointC = *msg;
                ROS_INFO("Changed the point C");
                _create_marker(0, 0, 1, 1, false, points_initialized, pointC);
            break;
            case 4:
                pointD = *msg;
                ROS_INFO("Changed the point D");
                _create_marker(0.6,0,0.30,1, false, points_initialized, pointD);
            break;
            case 5:
                pointE = *msg;
                ROS_INFO("Changed the point E");
                _create_marker(0.5, 0.5, 0.5, 1, false, points_initialized, pointE);
            break;
            case 6:
                pointF = *msg;
                ROS_INFO("Changed the point F");
                _create_marker(1, 0.4, 1, 1, false, points_initialized, pointF);
            break;
        }
        if (points_initialized < 7)
            points_initialized += 1;
    }

    //The program that listens on the chatter node, hears the other program and then through a series of if statements decide which goal to send
    void _listener_cb(const std_msgs::Char::ConstPtr& msg, const std_msgs::Char::ConstPtr& msg2)
    {
        ROS_INFO("I heard : [%c]", msg->data);

        switch(msg->data) {
              case 'A' : 
                ROS_INFO("Setting goal to A");
                _send_goal(pointA);
                break;
              case 'B' : 
                  ROS_INFO("Setting goal to B");
                  _send_goal(pointB);
                  break;
              case 'C' : 
                  ROS_INFO("Setting goal to C");
                  _send_goal(pointC);
                  break;
              case 'D' : 
                  ROS_INFO("Setting goal to D");
                  _send_goal(pointD);
                  break;
              case 'E' : 
                  ROS_INFO("Setting goal to E");
                  _send_goal(pointE);
                  break;
              case 'F' : 
                  ROS_INFO("Setting goal to F");
                  _send_goal(pointA);
                  break;
            default :
                    ROS_INFO("Something went wrong!");
        }
    }
    
//This is the nodes used for the program, and where the nodes are used
public:
    Route() :
        client("move_base"), points_initialized(0)
    {
        ros::NodeHandle n;
        marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
        
        chatter2_pub = n.advertise<std_msgs::String>("chatter2", 1);
        if (points_initialized < 7)
        click_sub = n.subscribe( "clicked_point", 100, &Route::_clicked_point_cb, this);

        chatter_sub = n.subscribe( "chatter", 10, &Route::_listener_cb, this);
    };
    ~Route(){};
};

// This is the main program - the program starts here
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navigation");

    Route r;
    
    ros::spin();
    return 0;
}