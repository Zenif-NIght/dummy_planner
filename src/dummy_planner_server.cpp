#include "ros/ros.h"

// Include the navigation messages GetPlan and Path
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"
// #include "nav_msgs/Path.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

/*!
 A service server which creates a plan consisting of the starting and end points
 \param req The service request
 \param res The planned path
 */
bool createDummyPlan(nav_msgs::GetPlan::Request & req,
                     nav_msgs::GetPlan::Response & res){//,
                    //  tf2_ros::Buffer &tf_buffer) {
    // Create the plan header
    res.plan.header.stamp = ros::Time::now();               // Set the time stamp to the current ros time
    res.plan.header.frame_id = req.start.header.frame_id;   // Use the frame from the starting point
    
    // Add the starting point
    res.plan.poses.emplace_back(req.start);

    // Add the ending point
    res.plan.poses.emplace_back(req.goal);

    return true;
}

/*!
  The main function will advertise a service and then spin
  */
int main(int argc, char **argv) {
    // Initialize ros and create a node handle
    ros::init(argc, argv, "planner_server_node");
    ros::NodeHandle n;

    // Advertise the service
    ros::ServiceServer service = n.advertiseService("dummy_plan", createDummyPlan);

	ROS_INFO("Ready to make a plan with two Points.");
	// Spin to wait for service requests
    ros::spin();
}
