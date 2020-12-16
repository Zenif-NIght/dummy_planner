#ifndef CONTINUOUS_PLANNER_HPP
#define CONTINUOUS_PLANNER_HPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Dense>

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"

#include <string>

namespace dummy_planner {
class ContinuousPlanner {
public:
    ContinuousPlanner(const std::string & map_frame_id);

    /*!
     * \brief odomCallback This function will be used to store odometry messages
     * \param msg Latest message
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /*!
     * \brief goalCallback This function will be used to store the goal messages
     * \param msg Latest message
     */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

     
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /*!
     * \brief getLatestPose Uses the latest odometry message to populate the pose information
     * \param pose Latest pose of the vehicle
     * \return true if the pose is populated, false otherwise
     */
    bool getLatestPose(geometry_msgs::PoseStamped & pose);

    /*!
     * \brief occupancyCallback Returns the latest OccupancyGrid in the map frame
     * \param msg Latest goal pose
     * \return
     */
    void occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    
    /*!
     * \brief getLatestGoal Returns the latest goal pose in the map frame
     * \param pose Latest goal pose
     * \return
     */
    bool getLatestGoal(geometry_msgs::PoseStamped & pose);
    
    /*!
     * \brief getLatestScan Returns the latest scan
     * \param scan Latest laser scan
     * \return
     */
    bool getLatestScan(sensor_msgs::LaserScan &scan);

    /*!
     * \brief map_frame_id the frame_id of the map
     */
    const std::string map_frame_id;

    /*!
     * \brief DoVectorField enable/disable vector field control
     * \param f boolean flag; true = enable vector field
     */
    void DoVectorField(bool f) { m_do_vector_field = f; }

    /*!
     * \brief DoVectorField enable/disable vector field control
     * \return return boolean flag; true = vector field enabled
     */
    bool VectorFieldEnabled() { return m_do_vector_field; }
    void DoAStar(bool f) { m_do_astar = f; }
    bool AStarEnabled() { return m_do_astar; }

    // /*!
    //  * \brief Pose2Vector2d convert a pose to an Eigen Vector2d
    //  */
    // Eigen::Vector2d Pose2Vector2d(const geometry_msgs::PoseStamped & pose);

    // /*!
    //  * \brief Vector2d2Pose convert an Eigen Vector2d to a Pose
    //  */
    // geometry_msgs::PoseStamped Vector2d2Pose(const Eigen::Vector2d & vec, const geometry_msgs::PoseStamped &ptemplate);
private:


    bool toMapFrame( geometry_msgs::PoseStamped &pose);
    
    /*!
     * \brief m_do_vector_field flag to enable vector field control
     */
    bool m_do_vector_field;
    bool m_do_astar;

    /*!
     * \brief m_last_map_size number of edges found
     */
    int m_last_map_size;

    /*!
     * \brief latest_odom is a pointer to the latest odometry message received
     */
    nav_msgs::Odometry::ConstPtr m_latest_odom;

    /*!
     * \brief m_latest_goal is the latest goal to plan to, stored in the correct frame
     */
    geometry_msgs::PoseStamped::ConstPtr m_latest_goal;

    /*!
     * \brief m_latest_scan stores the scan data
     */
    sensor_msgs::LaserScan::ConstPtr m_latest_scan;

    /*!
     * \brief latest_transformed_goal stores the transformed goal
     */
    geometry_msgs::PoseStamped m_latest_transformed_goal;

    /*!
     * \brief latest OccupancyGrid
     */
    nav_msgs::OccupancyGrid m_occupancy_grid;

    /*!
     * \brief flag_goal_transformed flag storing whether or not the goal has been transformed
     */
    bool m_flag_goal_transformed;

    /*!
     * \brief tf_buffer The transform buffer used to make transforms
     */
    tf2_ros::Buffer m_tf_buffer;

    /*!
     * \brief tf_listener The transform listener
     */
    tf2_ros::TransformListener m_tf_listener;
};

}

#endif  // CONTINUOUS_PLANNER_HPP
