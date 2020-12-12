#include "./continuous_planner.hpp"

#include "nav_msgs/GetPlan.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
//#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include "SummedFields.hpp"
#include "BetterUnicycleVehicle.hpp"
#include "RangeSensor.hpp"
#include "CombinedGoToGoalOrbitAvoidWithBarrierScenario.hpp"
// #include "vectorFollowingTypePoint.hpp"
#include "vectorFollowingTypePoint.cpp"
#include "control_type.hpp"

#include <eigen3/Eigen/Dense>

using namespace dummy_planner;
using namespace Eigen;
using namespace std;

ContinuousPlanner::ContinuousPlanner(const string & map_frame_id) :
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
    m_latest_scan = msg;
}

void ContinuousPlanner::occupancyCallback(const nav_msgs::OccupancyGridConstPtr& msg) { //OccupancyGrid::ConstPtr
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;
    // ROS_INFO("Got map %d %d", info.width, info.height);
    // std::vector<std::vector<int>> newMap(info.height,info.width );
    Eigen::MatrixXd newMap(info.height,info.width);
    for(int row=0; row<msg->info.width; row++)
    {
        for(int col =0; col<msg->info.height; col++)
        {
            int current_cell_value = msg->data[row+ (msg->info.width * col)];
            newMap(row,col) = current_cell_value;
        }
    }
    std::stringstream str;
    str << msg->info.width <<","<< msg->info.height<<"\r";
    str.str();
    // ROS_INFO_STREAM("Cur map size: "<< newMap); // 91 x 91

    // given latest Goal
    m_latest_goal;
    // for(int i=0; i<info.height*info.width; i++)
    //     {
    //         // if(msg->data[i]==-1)
    //         //     Ngray++;
    //         m_occupancy_grid.data[i] = msg->data[i];
    //     }
        
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
    // geometry_msgs::Vector3 diff1;
    // diff1.x = pnt2.pose.position.x - pnt1.pose.position.x;
    // diff1.y = pnt2.pose.position.y - pnt1.pose.position.y;
    // diff1.z = pnt2.pose.position.z - pnt1.pose.position.z;
    // double dist1 = sqrt( diff1.x*diff1.x + diff1.y*diff1.y + diff1.z*diff1.z );
    Vector2d diff = vec_goal - vec_start;
    double dist = diff.norm();
    // ROS_INFO("dist1: %f, dist: %f", dist1, dist);

    // If distance is less than the look_ahead then we only care about point 2
    if (dist < look_ahead) {
        result = pnt2;
        return;
    }

    // Get the unit vector between the two points
    // geometry_msgs::Vector3 unit1;
    // unit1.x = diff1.x / dist;
    // unit1.y = diff1.y / dist;
    // unit1.z = diff1.z / dist;
    Vector2d unit = diff / dist;
    // ROS_INFO("unit1: %f %f",unit1.x,unit1.y);
    // ROS_INFO("unit : %f %f",unit.x(),unit.y());

    // Calculate the new point
    // geometry_msgs::PoseStamped result1;
    // geometry_msgs::PoseStamped result1;
    // result1 = pnt1;
    // result1.pose.position.x += unit1.x*look_ahead;
    // result1.pose.position.y += unit1.y*look_ahead;
    // result1.pose.position.z += unit1.z*look_ahead;
    Vector2d res;
    res = vec_start + unit*look_ahead;
    result = Vector2d2Pose(res,pnt1);
    // ROS_INFO("result1 position: %f %f %f",result1.pose.position.x, result1.pose.position.y, result1.pose.position.z);
    // ROS_INFO("result position: %f %f %f",result.pose.position.x, result.pose.position.y, result.pose.position.z);
    // //ROS_INFO("result position: "+result.pose.position);

    // integrateEuler
    // Get orientation angle, theta
    double theta = tf::getYaw(pnt1.pose.orientation);
    // ROS_INFO_STREAM(" theta = " << theta);
    // ROS_INFO_STREAM("start position " << pnt1.pose.position);
    
    // Put current orientation into vehicle state
    scenario->setOrientation(pnt1.pose.position.x,pnt1.pose.position.y,theta);
    scenario->getObstacleDetections(scan);
    
    // Compute control inputs
    // TODO: time
    int t=0;
    Vector2d u = scenario->control(t,scenario->x_state());
    ROS_INFO_STREAM("cp: u = "<<u);

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

            if (!scenario)
            {
                ROS_INFO("Building a new Combined Orbit Avoid object...");
                RangeSensor sens(scan);
                vectorFollowingTypePoint control;
                // control_type control = vectorFollowingTypePoint();
                ////ROS_INFO_STREAM("sensor size: "<<sens.n_lines());
                // BetterUnicycleKinematics kin();
                BetterUnicycleVehicle veh(sens,&control);
                //control_type vctl = vectorFollowingTypePoint(veh);
                scenario = new CombinedGoToGoalOrbitAvoidWithBarrierScenario(
                                    // BetterUnicycleVehicle(BetterUnicycleKinematics(),
                                    //                     RangeSensor(scan),
                                    //                     vectorFollowingTypePoint()),
                                    veh,
                                    Pose2Vector2d(srv.request.goal));
            }
            // get the look ahead point 
            calculateLookAheadPoint(srv.request.start,
                                srv.request.goal,
                                look_ahead_dub,
                                scan,
                                scenario, 
                                look_ahead_point);


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
