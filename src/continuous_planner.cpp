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

#include <eigen3/Eigen/Dense>

using namespace dummy_planner;
using namespace Eigen;
using namespace std;

ContinuousPlanner::ContinuousPlanner(const string & map_frame_id) :
    map_frame_id(map_frame_id),
    m_flag_goal_transformed(false),
    m_tf_listener(m_tf_buffer),
    m_last_map_size(0),
    GLOBAL_path(LocListd()),
    GLOBAL_index(0)
{}

const int VECTOR_FILD_CONFIG = 0;
const int A_STAR_CONFIG = 1;
const int DIJKSTRA_CONFIG = 2;
const int BEST_FIRST_CONFIG = 3;

int GLOBLE_PLANNING_PARAMETER =1;

double LOOKAHEAD = 0.0;
void reconfigureCallback(dummy_planner::LookAheadConfig & config, uint32_t level){
    ROS_INFO("Reconfigure Requested ");//%d", config.int_param, config.double_param, );
    LOOKAHEAD = config.LookAhead;
    switch (config.plannning_parameter)
    {
    case 0:
        ROS_INFO("Reconfigure Requested Vector_fild");
        GLOBLE_PLANNING_PARAMETER = 0;
        break;
    case 1:
        ROS_INFO("Reconfigure Requested A_Star");
        GLOBLE_PLANNING_PARAMETER = 1;
        break;
    case 2:
        ROS_INFO("Reconfigure Requested Dijkstra");
        GLOBLE_PLANNING_PARAMETER = 2;
        break;
    case 3:
        ROS_INFO("Reconfigure Requested Best_First");
        GLOBLE_PLANNING_PARAMETER = 3;
        break;
    default:
        break;
    }
}

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
    // if(!msg || m_latest_odom != msg)return;
    // if no odom or goal, exit - wait until they are available

    // ROS_INFO("@@@@@ occCallback @@@@");

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


    if( (cur_map_size <= m_last_map_size) && (m_last_planned_goal == m_latest_goal) ) return;

    m_last_planned_goal = m_latest_goal;
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
    if (!getLatestPose(odom_pose)) return; // return if transform failure
    geometry_msgs::PoseStamped goal_pose;
    if (!getLatestGoal(goal_pose)) return; // return if transform failure

    // toMapFrame(m_latest_odom->pose.pose);
    // toMapFrame(m_latest_goal);

    Vector2d cur = Pose2Vector2d(odom_pose);
    Vector2d goal = Pose2Vector2d(goal_pose);
    Vector2d origin(info.origin.position.x,info.origin.position.y);
    cur = ( cur - origin ) / info.resolution;
    goal = ( goal - origin ) / info.resolution;
    // Vector2d cur(odom_pose.pose.position.x,odom_pose.pose.position.y);
    // Vector2d goal(goal_pose.pose.position.x,goal_pose.pose.position.y);

    // Vector2d cur((int)((m_latest_odom->pose.pose.position.x - info.origin.position.x)/info.resolution),
    //              (int)((m_latest_odom->pose.pose.position.y - info.origin.position.x)/info.resolution) );
    

    // Vector2d goal((int)((m_latest_goal->pose.position.x - info.origin.position.x)/info.resolution),
    //               (int)((m_latest_goal->pose.position.y - info.origin.position.x)/info.resolution));

    // ROS_WARN_STREAM("cur: ("<<m_latest_odom->pose.pose.position.x<<","<<m_latest_odom->pose.pose.position.y<<")");
    // ROS_WARN_STREAM("goal: ("<<m_latest_goal->pose.position.x<<","<<m_latest_goal->pose.position.y<<")");
    ROS_WARN_STREAM("\"map\" frame cur: ("<<cur(0)<<","<<cur(1)<<")");
    ROS_WARN_STREAM("\"map\" frame goal: ("<<goal(0)<<","<<goal(1)<<")");

    if (cur == goal ) 
    {
        // ROS_WARN("---- occCallback exit - Goal reached! (%f,%f)",goal(0),goal(1));
        return;
    } 

    // ROS_INFO_STREAM("cur: ("<<(unsigned int)(( cur(0) - info.origin.position.x)/info.resolution)<<","<< (unsigned int)(( cur(1) - info.origin.position.x)/info.resolution)<<")");

    ROS_INFO_STREAM("info.origin.position: ("<< info.origin.position.x<<","<<info.origin.position.y<<")");

    Vector2i icur((int)cur(0),(int)cur(1));
    Vector2i igoal((int)goal(0),(int)goal(1));
    AStarPlanner aStar_planner(newMap,
                               icur,
                               igoal);

    // vector<Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> path = aStar_planner.run_astar();
    GLOBAL_path = ContinuousPlanner::LocListd();
    aStar_planner.set_open_flag(true);
    aStar_planner.set_loop_flag(true);
    aStar_planner.set_path_flag(true);
    aStar_planner.set_max_depth(msg->info.width*msg->info.height);
    
    switch (GLOBLE_PLANNING_PARAMETER)
    {
    case VECTOR_FILD_CONFIG:
        return;
        break;        
    case A_STAR_CONFIG:
        aStar_planner.use_g_cost(true);
         aStar_planner.use_h_cost(true);
        break;
        case DIJKSTRA_CONFIG:
        aStar_planner.use_g_cost(true);
        aStar_planner.use_h_cost(false);
        break;
    case BEST_FIRST_CONFIG:
        aStar_planner.use_g_cost(false);
        aStar_planner.use_h_cost(true);
        break;
    
    default:
        
        break;
    }
    convertFrame(GLOBAL_path, aStar_planner.run_astar(),
                 origin,
                 info.resolution);
    if((int)GLOBAL_path.size() >=1){
        GLOBAL_index = 1;
    }
    else{
        GLOBAL_index = 0;
    }
    ROS_WARN("--- occCallback exit - GLOBAL_LOOKaHEAD path size: %d, index: %d",(int)GLOBAL_path.size(),GLOBAL_index); 
    m_latest_odom = nav_msgs::Odometry::ConstPtr();
    m_latest_goal = geometry_msgs::PoseStamped::ConstPtr();
}

void ContinuousPlanner::convertFrame(LocListd&newpath, const AStarPlanner::LocList &path, Vector2d mapPos, double resolution){
    ROS_INFO_STREAM("aStar_planner.run_astar==> convetion path.size():"<<path.size()); 
    // geometry_msgs::PoseStamped point;
    // point.header.stamp = m_latest_goal->header.stamp;
    // point.header.frame_id = m_latest_goal->header.frame_id;
    // point.pose = m_latest_goal->pose;

    for (int i = 0; i < path.size(); i++)
    {
        ROS_INFO_STREAM("PRE path["<<i<<"] " << path[i](0)<<","<<path[i](1));

        Vector2d pathd((double)path[i](0),(double)path[i](1));
        // toMapFrame(geometry_msgs::PoseStamped())
        newpath.push_back( pathd*resolution + mapPos );

        ROS_INFO_STREAM("POST path["<<i<<"] " << newpath[i](0)<<","<<newpath[i](1));


    }
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
Vector2d ContinuousPlanner::Pose2Vector2d(const geometry_msgs::PoseStamped &pose)
{
    return Vector2d(pose.pose.position.x, pose.pose.position.y);
}

// Get a Pose (stamped) from an Eigen Vector
geometry_msgs::PoseStamped ContinuousPlanner::Vector2d2Pose(const Vector2d &vec, const geometry_msgs::PoseStamped &ptemplate)
{
    geometry_msgs::PoseStamped pose = ptemplate;
    pose.pose.position.x = vec.x();
    pose.pose.position.y = vec.y();
    return pose;
}

void ContinuousPlanner::calculateLookAheadPoint(const geometry_msgs::PoseStamped & pnt1, // start position
                            const geometry_msgs::PoseStamped & pnt2, // goal position
                            double look_ahead,
                            const sensor_msgs::LaserScan &scan,
                            Scenario *scenario,
                            geometry_msgs::PoseStamped & result)
{
    Vector2d vec_start = Pose2Vector2d(pnt1), 
             vec_goal = Pose2Vector2d(pnt2);
    
    // ROS_INFO("############# calc LookAhead, Current:(%f,%f) Goal:(%f,%f) ###",vec_start(0),vec_start(1),vec_goal(0),vec_goal(1));
    
    // Calculate the distance between points
    Vector2d diff = vec_goal - vec_start;
    double dist = diff.norm();

    // If distance is less than the look_ahead then we only care about point 2
    if (dist < look_ahead) {
        result = pnt2;
        // ROS_INFO(".......... calc LookAhead exit - within goal; dist: %f, look_ahead limit: %f .....",dist,look_ahead);
        return;
    }

    // Get the unit vector between the two points
    Vector2d unit = diff / dist;

    // Calculate the new point
    Vector2d res;
    res = vec_start + unit*look_ahead;
    
    if (GLOBLE_PLANNING_PARAMETER == A_STAR_CONFIG)
    {
        // ROS_INFO("AStar enabled");
        //TESTING TODO FIX THIS
        if(GLOBAL_index >= (int)GLOBAL_path.size())
        {
            result = Vector2d2Pose(res,pnt1);
            // int end = GLOBAL_path.size()-1;
            // ROS_WARN("...... calc LookAhead exit - end of A* path reached. At goal? ");
            // ROS_WARN("  Current: [%f,%f];  ",vec_start[0],vec_start[1]);
            // ROS_WARN("  GLOBAL_index: %d  GLOBAL_path size: %d",GLOBAL_index,(int)GLOBAL_path.size());
            // if(GLOBAL_path.size() > 0) {
            //     ROS_WARN("  A* Begin: (%f,%f)  ",GLOBAL_path[0](0),GLOBAL_path[0](1));
            //     ROS_WARN("  Goal: (%f,%f)",GLOBAL_path[end](0),GLOBAL_path[end](1));
            // }
            return;
        }

        Vector2d next_pt =GLOBAL_path[GLOBAL_index]; ;
        //TODO if  GLOBAL_LOOKaHEAD is close to vec_start get next goal Point from path
        if( (next_pt - vec_start).norm() < look_ahead)
        {
            GLOBAL_index ++;
            next_pt = GLOBAL_path[GLOBAL_index];
            ROS_WARN_STREAM("NEW next_pt " << next_pt(0)<<","<<next_pt(1)<<"#################");

        }

        // ROS_INFO_STREAM("SET next_pt " << next_pt(0)<<","<<next_pt(1)<<"###");

        res = next_pt;
        result = Vector2d2Pose(res,pnt1);
        // ROS_INFO("result1 position: %f %f %f",result1.pose.position.x, result1.pose.position.y, result1.pose.position.z);
        // ROS_INFO("result position: %f %f %f",result.pose.position.x, result.pose.position.y, result.pose.position.z);
        // //ROS_INFO("result position: "+result.pose.position);
    }

    if (GLOBLE_PLANNING_PARAMETER == VECTOR_FILD_CONFIG)
    {
        // Vector Field 'Integration'
        // Get orientation angle, theta
        double theta = tf::getYaw(pnt1.pose.orientation);
        // Put current orientation into vehicle state
        ROS_INFO("loop: Insert current orientation");
        scenario->setOrientation(pnt1.pose.position.x,pnt1.pose.position.y,theta);
        // Read latest scan data into model
        ROS_INFO("loop: detect obstacles");
        scenario->getObstacleDetections(scan);
        
        // Compute control inputs
        // TODO: time
        int t=0;
        ROS_INFO("loop: get control inputs");
        vector<double> x_state = scenario->x_state();
        Vector2d u = scenario->control(t,x_state);
        ROS_INFO_STREAM("loop: control u = "<<u);

        // xdot = obj.vehicle.kinematics.kinematics(t, obj.vehicle.x, u);
        // vector<double> xdot = scenario->Kinematics(t,x_state, u); 

        // % Update the state
        // obj.vehicle.x = obj.vehicle.x + obj.dt * xdot;
        // vector<double> new_x = x_state + dt * xdot;
        // scenario->update_state(new_x);
        
        // Set look-ahead based on new state.

    }
    // ROS_INFO("exit main");
    // ROS_INFO("....... calcLookAhead exit - normal, next step: (%f,%f)",res(0),res(1));
    return;
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


     // create dynamic_reconfigure Server
    dynamic_reconfigure::Server<dummy_planner::LookAheadConfig> reconfig_server; 
    dynamic_reconfigure::Server<dummy_planner::LookAheadConfig>::CallbackType f;
    dummy_planner::LookAheadConfig default_config;
    default_config.LookAhead = look_ahead_dub;
    default_config.plannning_parameter =1;
    LOOKAHEAD = look_ahead_dub;
    reconfig_server.setConfigDefault(default_config);

    f = boost::bind(& reconfigureCallback, _1, _2);
    reconfig_server.setCallback(f);


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

            if (GLOBLE_PLANNING_PARAMETER == VECTOR_FILD_CONFIG)
            {
                if (!scenario)
                {
                    ROS_INFO("Building a new Combined Orbit Avoid object...");
                    RangeSensor sens(scan); // range sensor handler
                    vectorFollowingTypePoint control; // control field
                    BetterUnicycleKinematics kin; // vehicle kinematics
                    BetterUnicycleVehicle veh(kin,sens,&control); // vehicle
                    scenario = new CombinedGoToGoalOrbitAvoidWithBarrierScenario(
                                        veh, planner.Pose2Vector2d(srv.request.goal));
                    ROS_INFO("Done building Combined Orbit Avoid object");
                }
            }

            // get the look ahead point 
            planner.calculateLookAheadPoint(srv.request.start,
                                srv.request.goal,
                                LOOKAHEAD,//look_ahead_dub,
                                scan,
                                scenario, 
                                look_ahead_point);

            visualization_msgs::Marker line_strip;

            if (GLOBLE_PLANNING_PARAMETER == A_STAR_CONFIG 
                || GLOBLE_PLANNING_PARAMETER == DIJKSTRA_CONFIG
                || BEST_FIRST_CONFIG == GLOBLE_PLANNING_PARAMETER)
            {
                line_strip.header.frame_id =srv.request.start.header.frame_id;
                line_strip.header.stamp = srv.request.start.header.stamp;
                line_strip.ns =  "points_and_lines";
                line_strip.action = visualization_msgs::Marker::ADD;
                line_strip.pose.orientation.w =srv.request.start.pose.orientation.w; 
                line_strip.id = 1;
                line_strip.type = visualization_msgs::Marker::LINE_STRIP;
                line_strip.scale.x = 0.1;
                // Line strip is blue
                if(A_STAR_CONFIG == GLOBLE_PLANNING_PARAMETER)
                    line_strip.color.b = 1.0;
                if(DIJKSTRA_CONFIG == GLOBLE_PLANNING_PARAMETER)
                    line_strip.color.g = 1.0;
                if(BEST_FIRST_CONFIG == GLOBLE_PLANNING_PARAMETER)
                    line_strip.color.r = 1.0;
                line_strip.color.a = 1.0;
                line_strip.scale.x = 0.1;

                // line_strip.points.push_back(srv.request.start.pose.position);
                for (int i = planner.GLOBAL_index; i < planner.GLOBAL_path.size(); i++)
                {
                    geometry_msgs::Point p;
                    p.x = planner.GLOBAL_path[i](0);
                    p.y = planner.GLOBAL_path[i](1);
                    p.z = 0;
                    line_strip.points.push_back(p);
                }
                // line_strip.points.push_back(srv.request.goal.pose.position);
            }
          

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
