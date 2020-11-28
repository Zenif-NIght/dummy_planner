#ifndef CONTINUOUS_PLANNER_HPP
#define CONTINUOUS_PLANNER_HPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

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
     * \brief getLatestGoal Returns the latest goal pose in the map frame
     * \param pose Latest goal pose
     * \return
     */
    bool getLatestGoal(geometry_msgs::PoseStamped & pose);

    /*!
     * \brief map_frame_id the frame_id of the map
     */
    const std::string map_frame_id;

private:
    /*!
     * \brief latest_odom is a pointer to the latest odometry message received
     */
    nav_msgs::Odometry::ConstPtr m_latest_odom;

    /*!
     * \brief m_latest_goal is the latest goal to plan to, stored in the correct frame
     */
    geometry_msgs::PoseStamped::ConstPtr m_latest_goal;

    /*!
     * \brief latest_transformed_goal stores the transformed goal
     */
    geometry_msgs::PoseStamped m_latest_transformed_goal;

    /*!
     * \brief m_latest_scan stores the scan data
     */
    sensor_msgs::LaserScan::ConstPtr m_latest_scan;

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
