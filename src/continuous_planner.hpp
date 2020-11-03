#ifndef CONTINUOUS_PLANNER_HPP
#define CONTINUOUS_PLANNER_HPP

#include "ros/ros.h"

//TODO: Include the navigation messages odometry and path messages
#include ???
#include ???

namespace dummy_planner {
class ContinuousPlanner {
public:
    ContinuousPlanner(){}

    /*!
     * This function will be used to store odometry messages
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /*!
     * \brief getLatestPose Uses the latest odometry message to populate the pose information
     * \param pose Latest pose of the vehicle
     * \return true if the pose is populated, false otherwise
     */
    bool getLatestPose(geometry_msgs::PoseStamped & pose);

private:
    /*!
     * \brief latest_odom is a pointer to the latest odometry message received
     */
    nav_msgs::Odometry::ConstPtr latest_odom;
};

}

#endif  // CONTINUOUS_PLANNER_HPP