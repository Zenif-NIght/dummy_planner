#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"

/*!
 A service server which creates a plan consisting of the starting and end points
 \param req The service request
 \param res The planned path
 */
bool createDummyPlan(nav_msgs::GetPlan::Request & req,
                     nav_msgs::GetPlan::Response & res) {
    // Create the plan header
    res.plan.header.stamp = ros::Time::now();               // Set the time stamp to the current ros time
    res.plan.header.frame_id = req.start.header.frame_id;   // Use the frame from the starting point

    // Add the starting point
    res.plan.poses.emplace_back(req.start);

    // Add the ending poitn
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
    ros::ServiceServer service = n.advertiseService("final_plan", createDummyPlan);
    ros::spin();
}
