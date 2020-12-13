#pragma once

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <sensor_msgs/LaserScan.h>

#include "VehicleKinematics.hpp"
#include "Sensor.hpp"
#include "control_type.hpp"
#include "vectorFollowingTypePoint.hpp"

using namespace Eigen;
using namespace std;

class Vehicle
{
    // Vehicle is a simple abstract interface to enable construction of
    // different vehicles with different sensor configurations.
    // A Vehicle consists of the following elements:
    //   Kindematic model
    //   Controllers
    //      Vector Controller
    //      Path Controller
    //   Sensor - detects nearby obstacles
private:
    // Elements of the vehicle
    VehicleKinematics &m_kinematics;
    Sensor &m_sensor;
    control_type *m_control;

    // Vehicle state
    int m_t = 0; // latest time value
    vector<double> m_x; // state of the vehicle
    // Vector2d m_q; // position (this was buggy)
    int m_th_ind; // index of the orientation
    int m_x_ind; // x position
    int m_y_ind; // y position

    // void setQ();

    // Latest sensor meaurement
    vector<double> m_xo_latest; // obstacle x coords
    vector<double> m_yo_latest; // obstacle y coords
    vector<double> m_dist_latest; // distance to obstacle

public:
    Vehicle(VehicleKinematics &kin, Sensor &sens, control_type *type, const vector<double> &xo);
    // VehicleKinematics kinematics() { return &m_kinematics; }
    int x_ind() { return m_x_ind; }
    int y_ind() { return m_y_ind; }
    int th_ind() { return m_th_ind; }
    Vector2d q();
    double th() { return m_x[m_th_ind]; }
    double eps_vel() { return 0; }
    double vd_field_max() { return 0; }
    Matrix2d K_point_vel() { return Matrix2d(); }
    vector<double> x_state() { return m_x; }
    int n_sensors() { return m_sensor.n_lines(); }
    void setOrientation(double x, double y, double theta);
    void getObstacleDetections(const sensor_msgs::LaserScan &scan);
    Vector2d getObstacle(int k);
    Vector2d vectorFieldControl(int,field,const vector<double>&);
    Vector2d getVelocities(int t, const vector<double> &u);
};

// // set q, the coordinates or position
// void Vehicle::setQ()
// {
//     // ROS_INFO_STREAM("setQ build "<<m_x_ind<<","<<m_y_ind);
//     // // m_q << m_x[m_x_ind], m_x[m_y_ind];
//     // m_q(0) = m_x[m_x_ind];
//     // m_q(1) = m_x[m_y_ind];
//     // ROS_INFO("setQ done");
// }

// Constructor
Vehicle::Vehicle(VehicleKinematics &kin, Sensor &sens, control_type *type, const vector<double>& xo)
    : m_kinematics(kin), m_sensor(sens), m_control(type), m_x(xo)
{
    ROS_INFO("veh constructor");
    m_th_ind = m_kinematics.th_ind();
    m_x_ind = m_kinematics.x_ind();
    m_y_ind = m_kinematics.y_ind();
    vectorFollowingTypePoint *vp = dynamic_cast<vectorFollowingTypePoint*>(m_control);
    if (!vp) ROS_ERROR("control typoe not correct!");
    else vp->SayHi();
    // setQ();
    ROS_INFO("veh constructor done");
}

// Get q vector - [x,y] coordinates
Vector2d Vehicle::q()
{ 
    return Vector2d(m_x[m_x_ind],m_x[m_y_ind]);
}

// Set a new orientation (x,y, and theeta) from current robot position
void Vehicle::setOrientation(double x, double y, double theta)
{
    ROS_INFO("set orientation: %f,%f w:%f",x,y,theta);
    m_x[m_x_ind] = x;
    m_x[m_y_ind] = y;
    m_x[m_th_ind] = theta;
    // setQ();
    ROS_INFO_STREAM("veh orientation: x: "<<x<<" y: "<<y<<" theta "<<theta);
}

// pass laser scan data to sensor object for interpretation
void Vehicle::getObstacleDetections(const sensor_msgs::LaserScan &scan)
{   
    vector<vector<double>> obs = m_sensor.getObstacleDetections(q(), m_x[m_th_ind], scan);
    m_xo_latest = obs[0];
    m_yo_latest = obs[1];
    m_dist_latest = obs[2];
}

// Get the coordinates of the kth obstacle scan
Vector2d Vehicle::getObstacle(int k)
{
    Vector2d q;
    q(0) = m_xo_latest[k];
    q(1) = m_yo_latest[k];
    // TODO: detect "infinity" ??
    return q;
}

// ask the control object to get the next control inputs
Vector2d Vehicle::vectorFieldControl(int t,field f,const vector<double>&x)
{
    ROS_INFO("call control::vectorFieldControl()");
    vectorFollowingTypePoint *vp = dynamic_cast<vectorFollowingTypePoint*>(m_control);
    if (!vp) ROS_ERROR("control typoe not correct!");
    else return vp->vectorFieldControl(*this,t,f,x);
    ROS_INFO("  else");
    return m_control->vectorFieldControl(*this,t,f,x);
}

// Get [v,w] vector from kinematics
Vector2d Vehicle::getVelocities(int t, const vector<double> &u)
{
    m_kinematics.getVelocities(t,m_x,u);
}