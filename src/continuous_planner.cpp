#include "./continuous_planner.hpp"

#include "nav_msgs/GetPlan.h"

using namespace dummy_planner;

ContinuousPlanner::ContinuousPlanner(const std::string & map_frame_id) :
    m_map_frame_id(map_frame_id),
    m_flag_goal_transformed(false)
    // tf_listener(m_tf_buffer)
{}


void ContinuousPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    m_latest_odom = msg;
}
void ContinuousPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    m_latest_goal = msg;
    m_flag_goal_transformed = false;
}

bool ContinuousPlanner::getLatestPose(geometry_msgs::PoseStamped &pose,tf2_ros::Buffer &tf_buffer) {
    // Check to see if the pose has been received
    bool received = false; // Default is that no pose has been received
    if(m_latest_odom) {
        // Populate the header
        pose.header.stamp = m_latest_odom->header.stamp;
        pose.header.frame_id = m_latest_odom->header.frame_id;
        pose.pose = m_latest_odom->pose.pose;
        received = true; // Indicate that the pose was received
    }
    return received;
}

bool ContinuousPlanner::getLatestGoal(geometry_msgs::PoseStamped &pose) {
    // Check to see if the pose has been received
    bool received = false; // Default is that no pose has been received
    
    if(m_flag_goal_transformed) {
        pose = m_latest_transformed_goal;
        received = true;
    }
    else if(m_latest_goal) {
        // Populate the header
        pose.header.stamp = m_latest_goal->header.stamp;
        pose.header.frame_id = m_latest_goal->header.frame_id;
        pose.pose = m_latest_goal->pose;
        received = true; // Indicate that the pose was received
        m_flag_goal_transformed = true;
    }
    return received;
}

void calculateLookAheadPoint(const geometry_msgs::PoseStamped & pnt1, 
                            const geometry_msgs::PoseStamped & pnt2,
                            double look_ahead,
                            geometry_msgs::PoseStamped & result)
{
    // Calculate the distance between points
    geometry_msgs::Vector3 diff;
    diff.x = pnt2.pose.position.x - pnt1.pose.position.x;
    diff.y = pnt2.pose.position.y - pnt1.pose.position.y;
    diff.z = pnt2.pose.position.z - pnt1.pose.position.z;
    double dist = std::sqrt( diff.x*diff.x + diff.y*diff.y + diff.z*diff.z );

    // If distance is less than the look_ahead then we only care about point 2
    if (dist < look_ahead) {
        result = pnt2;
        return;
    }

    // Get the unit vector between the two points
    geometry_msgs::Vector3 unit;
    unit.x = diff.x / dist;
    unit.y = diff.y / dist;
    unit.z = diff.z / dist;

    // Calculate the new point
    result = pnt1;
    result.pose.position.x += unit.x*look_ahead;
    result.pose.position.y += unit.y*look_ahead;
    result.pose.position.z += unit.z*look_ahead;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_continuous_node");
    ros::start();
    ros::NodeHandle n;
    ros::NodeHandle nL("~");

    double look_ahead_dub =0.0;
    const std::string LOOK_AHEAD = "look_ahead";
    if (nL.hasParam(LOOK_AHEAD))
    {
        if (nL.getParam(LOOK_AHEAD, look_ahead_dub))
        {
            ROS_INFO("The service Param \"look_ahead\"was found");
        }
        else{
             ROS_WARN_THROTTLE(1, "The service Param \"look_ahead\" was found. BUT we could not get its value!");
        }
    }
    else{
        ROS_WARN_THROTTLE(1, "The service Param \"look_ahead\" was NOT be found!");
    }

    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);

	// Initialize the continuous planner
    ContinuousPlanner planner("map");
	
	// Create the subscription to the odometry message
    ros::Subscriber sub_odom = n.subscribe("odom", //robot1/odom
                                            1000,
                                            &ContinuousPlanner::odomCallback,
                                            &planner); 
    ros::Subscriber sub_goal = n.subscribe("goal", //robot1/odom
                                            1000,
                                            &ContinuousPlanner::goalCallback,
                                            &planner); 
    ROS_INFO("Ready to Subscriber to odom.");                                        

    // Create the publisher to publish the navigation path (use the n.advertize)
    ros::Publisher pub_plan = n.advertise<nav_msgs::Path>("path",1000); //robot1/path
    ROS_INFO("Ready to Publisher plan.");
    
    ros::Publisher pup_look_ahead_point = n.advertise<geometry_msgs::PoseStamped>("look_ahead_point",1000);
    geometry_msgs::PoseStamped look_ahead_point;    
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
                ROS_WARN_THROTTLE(1, "WARN 1: Could not transform to map frame: %s", ex.what());
            }
            try {
                pose_out = tf_buffer.transform<geometry_msgs::PoseStamped>(
                    srv.request.goal,
                    "map",
                    ros::Duration(1.0)
                    );
                planner.m_latest_transformed_goal = pose_out;
                srv.request.goal = pose_out;
                // get the look ahead point 
                calculateLookAheadPoint(srv.request.start,srv.request.goal,look_ahead_dub,look_ahead_point);

            } catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1, "WARN 2: Could not transform to map frame: %s", ex.what());
            }

            // Make the service request
            if(client.call(srv)) {
                // Publish the planned path
                pub_plan.publish(srv.response.plan);
                pup_look_ahead_point.publish(look_ahead_point); //geometry_msgs::PoseStamped
            } else {
                ROS_WARN_THROTTLE(1, "The service could not be connected");
            }

        } else {
            ROS_WARN_THROTTLE(1, "No odometry has been received");
        }
    }
}
