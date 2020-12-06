#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

#include "VehicleKinematics.hpp"
#include "Sensor.hpp"

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
    //TODO:
    VehicleKinematics m_kinematics;
    Sensor m_sensor;

    // Vehicle state
    int m_t = 0; // latest time value
    std::vector<double> m_x; // state of the vehicle
    Eigen::Vector2d m_q_ind; // indices of the position
    int m_th_ind; // index of the orientation
    int m_x_ind; // x position
    int m_y_ind; // y position

    // Latest sensor meaurement
    //TODO:
    // m_xo_latest;
    // m_yo_latest;
    // m_dist_latest;

public:
    Vehicle(VehicleKinematics kin, const std:vector<double> xo, const Eigen::Vector2d & q_ind);
    // Abstract
    //std::vector<double>& velocityControl(double vd, double wd) = 0;
    //std::vector<double>& pathControl(int t, ...);
    //std::vector<double>& vectorFieldControl(int t, const Eigen::Vector2d& g, control_type, ...);
}

Vehicle::Vehicle(VehicleKinematics kin, const std:vector<double>& xo, const Eigen::Vector2d& q_ind)
{
    m_kinematcis = kin;
    m_sensor = RangeSensor;
    m_x = xo;
    m_q_ind = q_ind;
    m_th_ind = m_kinematics.x_ind();
    m_x_ind = kinematics.x_ind();
    m_y_ind = kinematics.y_ind();
}