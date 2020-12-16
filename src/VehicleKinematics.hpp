#pragma once

#include "ros/ros.h"
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class VehicleKinematics
{
// VehicleKinematics Abstract class defining the required methods for a dynamics class

private:
    // State variables
    int m_x_ind = 0; // index into state of x position
    int m_y_ind = 1; // index into state of y osition
    int m_th_ind = 2; // index into state of theta - orientation
    int m_dimensions = 3; // number of dimensions (size) of state
    int m_n_u; // number of control inputs

public:
    VehicleKinematics(int dim);
    virtual int getNumberControlInputs();
    int x_ind() { return m_x_ind; }
    int y_ind() { return m_y_ind; }
    int th_ind() { return m_th_ind; }
    int dimensions() { return m_dimensions; }
    // Abstract methods
    //virtual vector<double> kinematics(int t, const vector<double>& x, const vector<double>& u);
    virtual Vector2d getVelocities(int t, const vector<double>& x, const vector<double>& u);
    vector<double> kinematics(int t, const vector<double> &x, const vector<double> &u);
};

// Constructor
VehicleKinematics::VehicleKinematics(int dem)
{
    m_dimensions = dem;
    m_n_u = getNumberControlInputs();
}

// number of control inputs - can be overridden by derived classes
int VehicleKinematics::getNumberControlInputs()
{
    // default is to return 2 control inputs
    return 2;
}

// vector<double> VehicleKinematics::kinematics(int t, const vector<double>& x, const vector<double>& u)
// {
//     // virtual function
//     vector<double> xdot;
//     return xdot;
// }

Vector2d VehicleKinematics::getVelocities(int t, const vector<double>& x, const vector<double>& u)
{
    // virtual function
    ROS_ERROR("VehicleKinematics getVelocities() base class invoked!");
    return Vector2d(0,0);
}

vector<double> VehicleKinematics::kinematics(int t, const vector<double> &x, const vector<double> &u)
{
    // virtual function
    ROS_ERROR("VehicleKinematics kinematics() base class invoked!");
    return vector<double>(m_dimensions,0);
}