#include "./continuous_planner.hpp"

#include "nav_msgs/GetPlan.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include "SummedFields.hpp"
#include "BetterUnicycleKinematics.hpp"
#include "BetterUnicycleVehicle.hpp"
#include "RangeSensor.hpp"
#include "CombinedGoToGoalOrbitAvoidWithBarrierScenario.hpp"
// #include "vectorFollowingTypePoint.hpp"
#include "vectorFollowingTypePoint.cpp"
#include "control_type.hpp"
#include <visualization_msgs/Marker.h>

#include "AStarPlanner.hpp"

#include <eigen3/Eigen/Dense>

using namespace dummy_planner;
using namespace Eigen;
using namespace std;

ContinuousPlanner::ContinuousPlanner(const string & map_frame_id) :
    map_frame_id(map_frame_id),
    m_flag_goal_transformed(false),
    m_tf_listener(m_tf_buffer),
    m_last_map_size(0)
{}

void ContinuousPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    m_latest_odom = msg;    
}

void ContinuousPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
    m_latest_goal = msg; // Store the goal
    m_flag_goal_transformed = false; // Indicate that the newest goal needs to be transformed
}

void ContinuousPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    m_latest_scan = msg;
}

AStarPlanner::LocList GLOBAL_path;
int GLOBAL_index = 0;
bool timeToGetNewPlan = false;

void ContinuousPlanner::occupancyCallback(const nav_msgs::OccupancyGridConstPtr& msg) { //OccupancyGrid::ConstPtr
    // if(!msg || m_latest_odom != msg)return;

    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;
    // ROS_INFO("Got map %d %d", info.width, info.height);
    // std::vector<std::vector<int>> newMap(info.height,info.width );
    Eigen::MatrixXi newMap(info.height,info.width);
    int cur_map_size = 0;
    
    for(int row=0; row<msg->info.width; row++)
    {
        for(int col =0; col<msg->info.height; col++)
        {
            int current_cell_value = msg->data[row+ (msg->info.width * col)];
            newMap(row,col) = current_cell_value;
            if(current_cell_value > 0)
                cur_map_size ++;    
        }
    }
    std::stringstream str;
    str << msg->info.width <<","<< msg->info.height<<"\r";
    str.str();
    // ROS_INFO_STREAM("Cur map size: "<< newMap); // 91 x 91

    // given latest Goal


    if( !m_latest_odom || !m_latest_goal || cur_map_size <= m_last_map_size )
        return; //no need to update Plan

    m_last_map_size = cur_map_size;
    ROS_INFO_STREAM("ccupancyCallback map_size: "<< m_last_map_size);                                         

    // Vector2d cur(m_latest_odom->pose.pose.position.x,
    //                  m_latest_odom->pose.pose.position.y);
    // Vector2d goal(m_latest_goal->pose.position.x,
    //                  m_latest_goal->pose.position.y);

 // https://answers.ros.org/question/10268/where-am-i-in-the-map/?answer=15060#post-id-15060
    // grid_x = (unsigned int)((map_x - map.info.origin.position.x) / map.info.resolution)
    // grid_y = (unsigned int)((map_y - map.info.origin.position.y) / map.info.resolution)
    geometry_msgs::PoseStamped odom_pose;
    geometry_msgs::PoseStamped goal_pose;

    toMapFrame(m_latest_odom->pose.pose);
    toMapFrame(m_latest_goal);

    Vector2d cur((int)((odom_pose->pose.pose.position.x - info.origin.position.x)/info.resolution),
                 (int)((odom_pose->pose.pose.position.y - info.origin.position.x)/info.resolution) );
    

    Vector2d goal((int)((goal_pose->pose.position.x - info.origin.position.x)/info.resolution),
                  (int)((goal_pose->pose.position.y - info.origin.position.x)/info.resolution));

    ROS_WARN_STREAM("cur: ("<<m_latest_odom->pose.pose.position.x<<","<<m_latest_odom->pose.pose.position.y<<")");
    ROS_WARN_STREAM("goal: ("<<m_latest_goal->pose.position.x<<","<<m_latest_goal->pose.position.y<<")");
    ROS_WARN_STREAM("cur: ("<<cur(0)<<","<<cur(1)<<")");
    ROS_WARN_STREAM("goal: ("<<goal(0)<<","<<goal(1)<<")");

    if (cur == goal )return; 

    // ROS_INFO_STREAM("cur: ("<<(unsigned int)(( cur(0) - info.origin.position.x)/info.resolution)<<","<< (unsigned int)(( cur(1) - info.origin.position.x)/info.resolution)<<")");

    ROS_INFO_STREAM("info.origin.position: ("<< info.origin.position.x<<","<<info.origin.position.y<<")");

    AStarPlanner aStar_planner(newMap,cur,goal);

    // vector<Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> path = aStar_planner.run_astar();
    GLOBAL_path = AStarPlanner::LocList();
    GLOBAL_path = aStar_planner.run_astar();

    aStar_planner.convertFrame(GLOBAL_path,
                        Vector2d(info.origin.position.x,info.origin.position.y),
                        info.resolution);
    // //TODO --> UPDATE LOOK AHEAD POINT 
    // for (int i = 0; i < 10; i++)
    // {
    //     GLOBAL_path.push_back(Vector2d(cur(0)+i,cur(1)+i));
    //     /* code */
    // }
    ROS_WARN_STREAM("GLOBAL_path: update");
    GLOBAL_index = 1;
    timeToGetNewPlan = true;
    ROS_WARN_STREAM("GLOBAL_LOOKaHEAD: " << GLOBAL_path[GLOBAL_index](0)<<","<<GLOBAL_path[GLOBAL_index](1)); 

}

bool ContinuousPlanner::toMapFrame( geometry_msgs::PoseStamped &pose){
   
    // Transform the goal pose to the latest map frame
    try {
        // Transform the goal
        pose = m_tf_buffer.transform<geometry_msgs::PoseStamped>(
            pose, 
            map_frame_id, 
            ros::Duration(1.0)
            );
        // Indicate that the goal has been received
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1, "WARN  ContinuousPlanner::toMapFrame() Could not transform to map frame: %s", ex.what());
        return false;
    }

    return false;
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

bool ContinuousPlanner::getLatestScan(sensor_msgs::LaserScan &scan) {
    // Check to see if the pose has been received
    bool received = false; // Default is that no pose has been received
    if(m_latest_scan) {
        scan = *m_latest_scan;
        received = true;
    }
    return received;
}

// Extract an Eigen Vector from Pose. Ignore the 3rd dimension
Vector2d Pose2Vector2d(const geometry_msgs::PoseStamped &pose)
{
    Vector2d vec;
    vec << pose.pose.position.x, pose.pose.position.y;
    return vec;
}

// Get a Pose (stamped) from an Eigen Vector
geometry_msgs::PoseStamped Vector2d2Pose(const Vector2d &vec, const geometry_msgs::PoseStamped &ptemplate)
{
    geometry_msgs::PoseStamped pose;
    pose = ptemplate;
    pose.pose.position.x = vec.x();
    pose.pose.position.y = vec.y();

    return pose;

}
void calculateLookAheadPoint(const geometry_msgs::PoseStamped & pnt1, // start position
                            const geometry_msgs::PoseStamped & pnt2, // goal position
                            double look_ahead,
                            const sensor_msgs::LaserScan &scan,
                            Scenario *scenario,
                            geometry_msgs::PoseStamped & result)
{
    Vector2d vec_start = Pose2Vector2d(pnt1), vec_goal = Pose2Vector2d(pnt2);
    
    // Calculate the distance between points
    Vector2d diff = vec_goal - vec_start;
    double dist = diff.norm();

    // If distance is less than the look_ahead then we only care about point 2
    if (dist < look_ahead) {
        result = pnt2;
        return;
    }

    // Get the unit vector between the two points
    Vector2d unit = diff / dist;

    // Calculate the new point
    Vector2d res;
    res = vec_start + unit*look_ahead;
    //TESTING TODO FIX THIS
    if(GLOBAL_index >= GLOBAL_path.size())
    {
        result = Vector2d2Pose(res,pnt1);
        return;
    }

    Vector2d LOOKaHEAD_pt =GLOBAL_path[GLOBAL_index]; ;
    //TODO if  GLOBAL_LOOKaHEAD is close to vec_start get next goal Point from path
    if( (LOOKaHEAD_pt - vec_start).norm() < 0.5)
    {
        GLOBAL_index ++;
        LOOKaHEAD_pt = GLOBAL_path[GLOBAL_index];
        ROS_WARN_STREAM("NEW LOOKaHEAD_pt " << LOOKaHEAD_pt(0)<<","<<LOOKaHEAD_pt(1)<<"#################");

    }

    // ROS_INFO_STREAM("SET LOOKaHEAD_pt " << LOOKaHEAD_pt(0)<<","<<LOOKaHEAD_pt(1)<<"###");

    res = LOOKaHEAD_pt;
    result = Vector2d2Pose(res,pnt1);
    // ROS_INFO("result1 position: %f %f %f",result1.pose.position.x, result1.pose.position.y, result1.pose.position.z);
    // ROS_INFO("result position: %f %f %f",result.pose.position.x, result.pose.position.y, result.pose.position.z);
    // //ROS_INFO("result position: "+result.pose.position);

    // integrateEuler
    // Get orientation angle, theta
    double theta = tf::getYaw(pnt1.pose.orientation);
    // ROS_INFO_STREAM(" theta = " << theta);
    // ROS_INFO_STREAM("start position " << pnt1.pose.position);
    return;
    // Put current orientation into vehicle state
    ROS_INFO("loop: Insert current orientation");
    scenario->setOrientation(pnt1.pose.position.x,pnt1.pose.position.y,theta);
    ROS_INFO("loop: detect obstacles");
    scenario->getObstacleDetections(scan);
    
    // Compute control inputs
    // TODO: time
    int t=0;
    ROS_INFO("loop: get control inputs");
    Vector2d u = scenario->control(t,scenario->x_state());
    ROS_INFO_STREAM("loop: control u = "<<u);

    // xdot = obj.vehicle.kinematics.kinematics(t, obj.vehicle.x, u);
    // VehicleKinematics need to find xdot 
    //(TOTO ADD DIFF_DRIVE kinematics)
    //(TODO ADD DIFF_DRIVE vehicle)

    // % Update the state
    // obj.vehicle.x = obj.vehicle.x + obj.dt * xdot;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "planner_continuous_node");
    ros::start();
    ros::NodeHandle n;
    ros::NodeHandle nL("~");

    double look_ahead_dub =0.0;
    const string LOOK_AHEAD = "look_ahead";
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

        // Create a subscription to the scan message
    ros::Subscriber occupancy_grid = n.subscribe("/occupancy_grid", 
                                            1, 
                                            &ContinuousPlanner::occupancyCallback,
                                            &planner);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_lines", 10);
    
    // TODO SET up CombinedGoToGoalOrbitAvoidWithBarrierScenario based off SCAN 
    
    ros::Publisher pub_plan = n.advertise<nav_msgs::Path>("path",1);                                        

    // Create the publisher to publish the navigation path
    // ros::Publisher pub_plan = n.advertise<nav_msgs::Path>("plan", 1);

    ros::Publisher pub_look_ahead_point = n.advertise<geometry_msgs::PoseStamped>("look_ahead_point",1000);
    geometry_msgs::PoseStamped look_ahead_point;   

    // Create the service client
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetPlan>("final_plan");

    // Create the scenario object, initially a null object, until all is ready
    Scenario *scenario = nullptr;

    // Run the program at 10 hz
    ros::Rate rate(10);
    while(ros::ok()) {
        // Process any callbacks
        ros::spinOnce();

        // Check inputs
        nav_msgs::GetPlan srv;
        bool goal_received = planner.getLatestGoal(srv.request.goal);
        bool pose_received = planner.getLatestPose(srv.request.start);
        sensor_msgs::LaserScan scan;
        bool scan_received = planner.getLatestScan(scan);
        // bool scan_received = planner.getLatestScan(srv.request.start);
        if(!goal_received) {
            ROS_WARN_THROTTLE(10, "No goal yet received");
        }
        if (!pose_received) {
            ROS_WARN_THROTTLE(10, "No odometry has been received");
        }
        if (!scan_received) {
            ROS_WARN_THROTTLE(10, "No scan has been received");
        }
 
        // Call the service to get the plan
        if(goal_received && pose_received && scan_received) {

            // if (!scenario)
            // {
            //     ROS_INFO("Building a new Combined Orbit Avoid object...");
            //     RangeSensor sens(scan);
            //     vectorFollowingTypePoint control;
            //     // control_type control = vectorFollowingTypePoint();
            //     ////ROS_INFO_STREAM("sensor size: "<<sens.n_lines());
            //     BetterUnicycleKinematics kin;
            //     BetterUnicycleVehicle veh(kin,sens,&control);
            //     //control_type vctl = vectorFollowingTypePoint(veh);
            //     scenario = new CombinedGoToGoalOrbitAvoidWithBarrierScenario(
            //                         // BetterUnicycleVehicle(BetterUnicycleKinematics(),
            //                         //                     RangeSensor(scan),
            //                         //                     vectorFollowingTypePoint()),
            //                         veh,
            //                         Pose2Vector2d(srv.request.goal));
            //     ROS_INFO("Done building Combined Orbit Avoid object");
            // }
            // get the look ahead point 
            calculateLookAheadPoint(srv.request.start,
                                srv.request.goal,
                                look_ahead_dub,
                                scan,
                                scenario, 
                                look_ahead_point);

            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id =srv.request.start.header.frame_id;
            line_strip.header.stamp = srv.request.start.header.stamp;
            line_strip.ns =  "points_and_lines";
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.pose.orientation.w =srv.request.start.pose.orientation.w; 
            line_strip.id = 1;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.scale.x = 0.1;
            // Line strip is blue
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;
            line_strip.scale.x = 0.1;

            // line_strip.points.push_back(srv.request.start.pose.position);
            for (int i = GLOBAL_index; i < GLOBAL_path.size(); i++)
            {
                geometry_msgs::Point p;
                p.x = GLOBAL_path[i](0);
                p.y = GLOBAL_path[i](1);
                p.z = 0;
                line_strip.points.push_back(p);
            }
            timeToGetNewPlan =false;
            // line_strip.points.push_back(srv.request.goal.pose.position);

          

            // Make the service request
            if(client.call(srv)) {
                // Publish the planned path
                pub_plan.publish(srv.response.plan);
                pub_look_ahead_point.publish(look_ahead_point); //geometry_msgs::PoseStamped
                marker_pub.publish(line_strip);
            } else {
                ROS_WARN_THROTTLE(1, "WARN 3: The service could not be connected");
            }
        }
    }
}
