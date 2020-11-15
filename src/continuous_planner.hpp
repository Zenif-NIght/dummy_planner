#ifndef CONTINUOUS_PLANNER_HPP
#define CONTINUOUS_PLANNER_HPP

#include "ros/ros.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"

#include "nav_msgs/Odometry.h"

namespace dummy_planner {
class ContinuousPlanner {
public:
    ContinuousPlanner(){}

    /*!
     * This function will be used to store odometry messages
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    void goalCallback(const geometry_msgs::PoseStamped& msg);
    
    /*!
     * \brief getLatestPose Uses the latest odometry message to populate the pose information
     * \param pose Latest pose of the vehicle
     * \return true if the pose is populated, false otherwise
     */
    bool getLatestPose(geometry_msgs::PoseStamped & pose, tf2_ros::Buffer &tf_buffer_hpp);

private:
    /*!
     * \brief m_latest_odom is a pointer to the latest odometry message received
     */
     nav_msgs::Odometry::ConstPtr m_latest_odom;
     geometry_msgs::PoseStamped m_latest_goal;
};

}

#endif  // CONTINUOUS_PLANNER_HPP
