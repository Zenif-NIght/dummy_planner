#include "./continuous_planner.hpp"

#include "nav_msgs/GetPlan.h"

using namespace dummy_planner;


void ContinuousPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    m_latest_odom = msg;
}
void ContinuousPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    m_latest_goal = msg;
}

bool ContinuousPlanner::getLatestPose(geometry_msgs::PoseStamped &pose,tf2_ros::Buffer &tf_buffer) {
    // Check to see if the pose has been received
    bool received = false; // Default is that no pose has been received
    if(m_latest_odom) {
        // Populate the header
        pose.header.stamp = m_latest_odom->header.stamp;
        pose.header.frame_id = m_latest_odom->header.frame_id;
		// try {
        //     pose_out = tf_buffer.transform<geometry_msgs::PoseStamped>(pose_in, frame_id, ros::Duration(1.0));
        // } catch (tf2::TransformException &ex) {
        //     ROS_WARN_THROTTLE(1, "Could not transform to map frame: %s", ex.what());
        // }
		// put the pose from the latest odometry into the pose variable
        pose.pose = m_latest_odom->pose.pose;
        received = true; // Indicate that the pose was received
    }
    // else {
    //     // if(m_latest_goal)
    //             // Populate the header
    //     pose.header.stamp = m_latest_goal.header.stamp;
    //     pose.header.frame_id = m_latest_goal.header.frame_id;
    //     ROS_INFO("frame_id: "+pose.header.frame_id);
    //     // ROS_INFO(pose.header.frame_id);
	// 	// try {
    //     //     pose_out = tf_buffer.transform<geometry_msgs::PoseStamped>(pose_in, frame_id, ros::Duration(1.0));
    //     // } catch (tf2::TransformException &ex) {
    //     //     ROS_WARN_THROTTLE(1, "Could not transform to map frame: %s", ex.what());
    //     // }
	// 	// put the pose from the latest odometry into the pose variable
    //     pose.pose = m_latest_goal.pose;
    //     received = true; // Indicate that the pose was received
    // }
    return received;
}

bool ContinuousPlanner::getLatestGoal(geometry_msgs::PoseStamped &pose) {
    // Check to see if the pose has been received
    bool received = false; // Default is that no pose has been received
    if(m_latest_goal) {
        // Populate the header
        pose.header.stamp = m_latest_goal->header.stamp;
        pose.header.frame_id = m_latest_goal->header.frame_id;
        pose.pose = m_latest_goal->pose;
        received = true; // Indicate that the pose was received
    }
    return received;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_continuous_node");
    ros::start();
    ros::NodeHandle n;

    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

	// Initialize the continuous planner
    ContinuousPlanner planner;
	
	// Create the subscription to the odometry message
    ros::Subscriber sub_odom = n.subscribe("odom", //robot1/odom
                                            1000,
                                            &ContinuousPlanner::odomCallback,//&ContinuousPlanner::odomCallback,//&ContinuousPlanner::goalCallback
                                            &planner); 
    ros::Subscriber sub_goal = n.subscribe("goal", //robot1/odom
                                            1000,
                                            &ContinuousPlanner::goalCallback,//&ContinuousPlanner::odomCallback,//&ContinuousPlanner::goalCallback
                                            &planner); 
    ROS_INFO("Ready to Subscriber to odom.");                                        

    // Create the publisher to publish the navigation path (use the n.advertize)
    ros::Publisher pub_plan = n.advertise<nav_msgs::Path>("path",1000); //robot1/path
    ROS_INFO("Ready to Publisher plan.");
    // Create the service request
    nav_msgs::GetPlan srv;

    // Create the service client
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("dummy_plan");

    // Run the program at 10 hz
    ros::Rate rate(10);
    while(ros::ok()) {
        // Process any callbacks
        ros::spinOnce();

        // Call the service to get the plan
        if(planner.getLatestPose(srv.request.start, tf_buffer) && planner.getLatestGoal(srv.request.goal)) { // check to see if an odometry message has come in
            // Update the header information for the goal
            geometry_msgs::PoseStamped pose_out;
            try {
                pose_out = tf_buffer.transform<geometry_msgs::PoseStamped>(
                    srv.request.start,
                    "map",
                    ros::Duration(1.0)
                    );
                srv.request.start = pose_out;
            } catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1, "Could not transform to map frame: %s", ex.what());
            }
            try {
                srv.request.goal = tf_buffer.transform<geometry_msgs::PoseStamped>(
                    srv.request.goal,
                    "map",
                    ros::Duration(1.0)
                    );
            } catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1, "Could not transform to map frame: %s", ex.what());
            }

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
