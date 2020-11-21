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
    ContinuousPlanner(const std::string & map_frame_id);

    /*!
     * This function will be used to store odometry messages
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    /*!
     * \brief getLatestPose Uses the latest odometry message to populate the pose information
     * \param pose Latest pose of the vehicle
     * \return true if the pose is populated, false otherwise
     */
    bool getLatestPose(geometry_msgs::PoseStamped & pose, tf2_ros::Buffer &tf_buffer_hpp);
    bool getLatestGoal(geometry_msgs::PoseStamped & pose);


    /*!
     * \brief map_frame_id the frame_id of the map
     */
    const std::string m_map_frame_id;

    //brief latest_transformed_goal stores the transformed goal
    geometry_msgs::PoseStamped m_latest_transformed_goal;

private:
    /*!
     * \brief m_latest_odom is a pointer to the latest odometry message received
     */
    nav_msgs::Odometry::ConstPtr m_latest_odom;
    geometry_msgs::PoseStamped::ConstPtr m_latest_goal;     
    //brief flag_goal_transformed flag storing whether or not the goal has been transformed
    bool m_flag_goal_transformed;

    // /*!
    //  * \brief tf_buffer The transform buffer used to make transforms
    //  */
    // tf2_ros::Buffer m_tf_buffer;

    // /*!
    //  * \brief tf_listener The transform listener
    //  */
    // tf2_ros::TransformListener m_tf_listener;


};

}

#endif  // CONTINUOUS_PLANNER_HPP
