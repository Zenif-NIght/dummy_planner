#include "./continuous_planner.hpp"

#include "nav_msgs/GetPlan.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "GoToGoalField.hpp"
#include "OrbitAvoidField.hpp"
#include "AvoidObstacle.hpp"
#include "SummedFields.hpp"

#include <eigen3/Eigen/Dense>

using namespace dummy_planner;

ContinuousPlanner::ContinuousPlanner(const std::string & map_frame_id) :
    map_frame_id(map_frame_id),
    m_flag_goal_transformed(false),
    m_tf_listener(m_tf_buffer)
{}

void ContinuousPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    m_latest_odom = msg;    
}

void ContinuousPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
    m_latest_goal = msg; // Store the goal
    m_flag_goal_transformed = false; // Indicate that the newest goal needs to be transformed
}
void ContinuousPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    latest_scan = msg;
}

bool ContinuousPlanner::getLatestGoal(geometry_msgs::PoseStamped & pose) {
    bool received = false; // Default to not received yet

    // Check to see if the goal has already been transformed
    if(m_flag_goal_transformed) {
        pose = m_latest_transformed_goal;
        received = true;
    }
    // Check to see if the goal has been received
    else if(m_latest_goal) {
        // Transform the goal pose to the latest map frame
        try {
            // Transform the goal
            m_latest_transformed_goal = m_tf_buffer.transform<geometry_msgs::PoseStamped>(
                *m_latest_goal, 
                map_frame_id, 
                ros::Duration(1.0)
                );
            m_flag_goal_transformed = true;
            pose = m_latest_transformed_goal;
            // Indicate that the goal has been received
            received = true;
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1, "WARN 1: ContinuousPlanner::getLatestGoal() Could not transform to map frame: %s", ex.what());
            received = false;
        }
    }
    return received;
}

bool ContinuousPlanner::getLatestPose(geometry_msgs::PoseStamped &pose) {
    // Check to see if the pose has been received
    bool received = false; // Default is that no pose has been received
    if(m_latest_odom) {
        // Create a stamped pose from the odometry
        geometry_msgs::PoseStamped odom;
        odom.header.stamp = m_latest_odom->header.stamp;
        odom.header.frame_id = m_latest_odom->header.frame_id;
        odom.pose = m_latest_odom->pose.pose;

        // Transform the odometry into the new map frame
        try {
            pose = m_tf_buffer.transform<geometry_msgs::PoseStamped>(
                odom,
                map_frame_id, 
                ros::Duration(1.0)
                );
            received  = true;
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1, "WARN 2: ContinuousPlanner::getLatestPose() Could not transform to map frame: %s", ex.what());
            received = false;
        }
    }
    return received;
}

// bool ContinuousPlanner::getLatestScan(sensor_msgs::LaserScan &scan) {
//     // Check to see if the pose has been received
//     bool received = false; // Default is that no pose has been received
//     if(m_latest_scan) {
//         scan = m_latest_scan;
//         return  m_latest_scan;
//         received = false;
//     }
//     return received;
// }

void calculateLookAheadPoint(const geometry_msgs::PoseStamped & pnt1, 
                            const geometry_msgs::PoseStamped & pnt2,
                            double look_ahead,
                            sensor_msgs::LaserScan::ConstPtr &scan,
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

    // Apply vector fileds to that point

    //look at all lazer points 

    // if cloes enuph do the orbit avoid 
    

    //         % Go to goal variables
    //     x_g =  [20; 5]; %[16; 10]; % try [20; 5]; [25; 5];
    // geometry_msgs::Point x_g;
    // x_g.x = result.pose.position.x;
    // x_g.y = result.pose.position.y;
    Eigen::Vector2d x_g;
    x_g << result.pose.position.x , result.pose.position.y;
        
    //     % Obstacle avoidance variables - orbit
    double S = 3; //% Sphere of influence
    double R = 2; // Radius of orbit
    double k_conv = 0.5; // Convergence gain
        
    //     % Obstacle avoidance variables - barrier
    double S_b = 1.0; // Sphere of influence of barrier
    double R_b = 0.5; // Radius of full influence
        
    //     % Weights
    double w_g2g = 1;
    double w_avoid = 1;
    double w_barrier = 10;
    // double weights = zeros(1+veh.sensor.n_lines*2, 1);
    if(!scan)
        return;

    int n_lines = scan->ranges.size() ;
    std::vector<double> weights(n_lines * 2 +1,1);
    weights[0] = w_g2g;
    // weights[2:n_lines] = w_avoid;
    // weights[2+n_lines:weights.end()] = w_barrier;
    for (int i=1; i < weights.size(); i++)
    {
        if(i > 1+n_lines)
            weights[i] = w_barrier;
        else
            weights[i] = w_avoid;
    }
    // % Create a weighted field for the vector field to follow
    // fields = cell(1+veh.sensor.n_lines*2, 1); % 1 for goal to goal and then the rest for the object avoidance
    std::vector<field> fields(weights.size());
    //     avoid_indices = 2:veh.sensor.n_lines+1;
    std::vector<int> avoid_indices(n_lines+1);
    for (int i = 0; i < n_lines+1; i++) {avoid_indices[i] = i+1;}
    
    // q_inf = [10000000; 10000000];
    Eigen::Vector2d q_inf;
    q_inf << 10000000 , 10000000;

    const double v_max = 2;
    //     fields{1} = GoToGoalField(x_vec, y_vec, x_g, v_max);
    fields[0] = GoToGoalField( x_g, v_max);
    // for k = avoid_indices
        // fields{k} = OrbitAvoidField(x_vec, y_vec, q_inf, R, v_max, k_conv, S);  
        // fields{k+veh.sensor.n_lines} = AvoidObstacle(x_vec, y_vec, q_inf, v_max);
    //     fields{k+veh.sensor.n_lines}.S = S_b;
    //     fields{k+veh.sensor.n_lines}.R = R_b;
    // end
    for (int i : avoid_indices)
    {
        fields[i] = OrbitAvoidField(q_inf, R, v_max, k_conv,S);
        fields[i+n_lines] = AvoidObstacle( q_inf, v_max, S_b, R_b);
    }  


    //     % Create a combined vector field
    //     field = SummedFields(x_vec, y_vec, fields, weights, v_max);
    field sfield = SummedFields(fields, weights, v_max);
    
    // //     % Create the scenario
    // //     obj = obj@VectorFieldScenario(field, veh, PolygonWorld1, control_type);
        
    // // %             veh.x(veh.q_ind) = [5; -1];
    // // %             veh.x(veh.th_ind) = pi/4;
        
    // //     % Store object variables
    // //     obj.avoid_indices = avoid_indices;
    // //     obj.q_inf = q_inf; 
    // //     obj.n_sensors = veh.sensor.n_lines;
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

    // Create the subscription to the odometry message
    ContinuousPlanner planner("map");
    ros::Subscriber sub_odom = n.subscribe("odom",
                                            1, 
                                            &ContinuousPlanner::odomCallback, 
                                            &planner);

    // Create a subscription to the goal message
    ros::Subscriber sub_goal = n.subscribe("goal", 
                                            1, 
                                            &ContinuousPlanner::goalCallback,
                                            &planner);

    // Create a subscription to the scan message
    ros::Subscriber sub_scan = n.subscribe("scan", 
                                            1, 
                                            &ContinuousPlanner::scanCallback,
                                            &planner);                                            

    ros::Publisher pub_plan = n.advertise<nav_msgs::Path>("path",1);                                        

    // Create the publisher to publish the navigation path
    // ros::Publisher pub_plan = n.advertise<nav_msgs::Path>("plan", 1);

    ros::Publisher pub_look_ahead_point = n.advertise<geometry_msgs::PoseStamped>("look_ahead_point",1000);
    geometry_msgs::PoseStamped look_ahead_point;   

    // Create the service client
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("final_plan");

    // Run the program at 10 hz
    ros::Rate rate(10);
    while(ros::ok()) {
        // Process any callbacks
        ros::spinOnce();

        // Check inputs
        nav_msgs::GetPlan srv;
        bool goal_received = planner.getLatestGoal(srv.request.goal);
        bool pose_received = planner.getLatestPose(srv.request.start);
        // bool scan_received = planner.getLatestScan(srv.request.start);
        if(!goal_received) {
            ROS_WARN_THROTTLE(10, "No goal yet received");
        }
        if (!pose_received) {
            ROS_WARN_THROTTLE(10, "No odometry has been received");
        }
        // if (!scan_received) {
        //     ROS_WARN_THROTTLE(10, "No scan has been received");
        // }

        // get the look ahead point 
        calculateLookAheadPoint(srv.request.start,
                                srv.request.goal,
                                look_ahead_dub,
                                planner.latest_scan,
                                look_ahead_point);


        // Call the service to get the plan
        if(goal_received && pose_received) {
            // Make the service request
            if(client.call(srv)) {
                // Publish the planned path
                pub_plan.publish(srv.response.plan);
                pub_look_ahead_point.publish(look_ahead_point); //geometry_msgs::PoseStamped
            } else {
                ROS_WARN_THROTTLE(1, "WARN 3: The service could not be connected");
            }
        }
    }
}
