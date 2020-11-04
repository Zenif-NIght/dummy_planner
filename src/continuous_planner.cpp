#include "./continuous_planner.hpp"

#include "nav_msgs/GetPlan.h"

using namespace dummy_planner;

void ContinuousPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    m_latest_odom = msg;
}

bool ContinuousPlanner::getLatestPose(geometry_msgs::PoseStamped &pose) {
    // Check to see if the pose has been received
    bool received = false; // Default is that no pose has been received
    if(m_latest_odom) {
        // Populate the header
        pose.header.stamp = m_latest_odom->header.stamp;
        pose.header.frame_id = m_latest_odom->header.frame_id;
		
		// put the pose from the latest odometry into the pose variable
        pose.pose = m_latest_odom->pose.pose;
        received = true; // Indicate that the pose was received
    }
    return received;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_continuous_node");
    ros::start();
    ros::NodeHandle n;

	// Initialize the continuous planner
    ContinuousPlanner planner;
	
	// Create the subscription to the odometry message
    ros::Subscriber sub_odom = n.subscribe("robot1/odom", 
                                            1000,
                                            &ContinuousPlanner::odomCallback,
                                            &planner); 
    ROS_INFO("Ready to Subscriber to odom.");                                        

    // Create the publisher to publish the navigation path (use the n.advertize)
    ros::Publisher pub_plan = n.advertise<nav_msgs::Path>("robot1/path",1000);
    ROS_INFO("Ready to Publisher plan.");
    // Create the service request
    nav_msgs::GetPlan srv;
    srv.request.goal.pose.position.x = 0;
    srv.request.goal.pose.position.y = 0;
    srv.request.goal.pose.position.z = 0;

    // Create the service client
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("dummy_plan");

    // Run the program at 10 hz
    ros::Rate rate(10);
    while(ros::ok()) {
        // Process any callbacks
        ros::spinOnce();

        // Call the service to get the plan
        if(planner.getLatestPose(srv.request.start)) { // check to see if an odometry message has come in
            // Update the header information for the goal
            srv.request.goal.header.frame_id = srv.request.start.header.frame_id;
            srv.request.goal.header.stamp = srv.request.start.header.stamp;

            // Make the service request
            if(client.call(srv)) {
                // Publish the planned path
                pub_plan.publish(srv.response.plan);
            } else {
                ROS_WARN_THROTTLE(1, "The service could not be connected");
            }

        } else {
            ROS_WARN_THROTTLE(1, "No odometry has been received");
        }
    }
}
