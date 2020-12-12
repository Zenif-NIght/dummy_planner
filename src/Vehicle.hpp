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
    VehicleKinematics m_kinematics;
    Sensor m_sensor;
    control_type *m_control;

    // Vehicle state
    int m_t = 0; // latest time value
    vector<double> m_x; // state of the vehicle
    Vector2d m_q; // position
    int m_th_ind; // index of the orientation
    int m_x_ind; // x position
    int m_y_ind; // y position

    void setQ();

    // Latest sensor meaurement
    vector<double> m_xo_latest; // obstacle x coords
    vector<double> m_yo_latest; // obstacle y coords
    vector<double> m_dist_latest; // distance to obstacle

public:
    Vehicle(VehicleKinematics kin, Sensor sens, control_type *type, const vector<double> &xo);
    VehicleKinematics kinematics() { return m_kinematics; }
    int x_ind() { return m_x_ind; }
    int y_ind() { return m_y_ind; }
    int th_ind() { return m_th_ind; }
    Vector2d q() { return m_q; }
    double th() { return m_x[m_th_ind]; }
    double eps_vel() { return 0; }
    double vd_field_max() { return 0; }
    Matrix2d K_point_vel() { return Matrix2d(); }
    vector<double> x_state() { return m_x; }
    int n_sensors() { return m_sensor.n_lines(); }
    void setOrientation(double x, double y, double theta);
    vector<vector<double>> getObstacleDetections(const sensor_msgs::LaserScan &scan);
    Vector2d getObstacle(int k);
    Vector2d vectorFieldControl(int,field,const vector<double>&);


    // Abstract
    //vector<double>& velocityControl(double vd, double wd) = 0;
    //vector<double>& pathControl(int t, ...);
    //vector<double>& vectorFieldControl(int t, const Vector2d& g, control_type, ...);
};

// set q_ind, the coordinates
void Vehicle::setQ()
{
    // ROS_INFO_STREAM("setQ build");
    // m_q << m_x[m_x_ind], m_x[m_y_ind];
    m_q[0] = m_x[m_x_ind];
    m_q[1] = m_x[m_y_ind];
    // ROS_INFO("setQ done");
}

Vehicle::Vehicle(VehicleKinematics kin, Sensor sens, control_type *type, const vector<double>& xo)
    : m_kinematics(kin), m_sensor(sens), m_control(type), m_x(xo)
{
    ROS_INFO("veh constructor");
    m_th_ind = m_kinematics.th_ind();
    m_x_ind = m_kinematics.x_ind();
    m_y_ind = m_kinematics.y_ind();
    setQ();
    ROS_INFO("veh constructor done");
}

void Vehicle::setOrientation(double x, double y, double theta)
{
    m_x[m_x_ind] = x;
    m_x[m_y_ind] = y;
    m_x[m_th_ind] = theta;
    setQ();
    ROS_INFO_STREAM("veh orientation: x: "<<x<<" y: "<<y<<" theta "<<theta);
}

vector<vector<double>> Vehicle::getObstacleDetections(const sensor_msgs::LaserScan &scan)
{   
    vector<vector<double>> obs = m_sensor.getObstacleDetections(m_q, m_x[m_th_ind], scan);
    m_xo_latest = obs[0];
    m_yo_latest = obs[1];
    m_dist_latest = obs[2];

    return obs;
}

// Get the coordinates of the kth obstacle scan
Vector2d Vehicle::getObstacle(int k)
{
    Vector2d q;
    q << m_xo_latest[k];
    q << m_yo_latest[k];
    // TODO: detect "infinity" ??
    return q;
}

Vector2d Vehicle::vectorFieldControl(int t,field f,const vector<double>&x)
{
    return m_control->vectorFieldControl(*this,t,f,x);
}
