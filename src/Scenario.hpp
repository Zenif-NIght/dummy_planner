#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

#include "Vehicle.hpp"
#include "field.hpp"

using namespace Eigen;
using namespace std;

class Scenario
{
private:
    // Define elements to be simulated
    Vehicle &m_vehicle; // instance of the Vehicle class

    // Simulation parameters
    int m_t_0; // initial time of simulation
    double m_dt = 0.05; // Simulation step size
    int m_tf = 25; // Final time of simulation

    // index variables
    int m_x_ind; // x position index
    int m_y_ind; // y position index
    Vector2d m_q_ind; // 2D position index

public:
    Scenario(Vehicle &veh);
    virtual Vector2d control(int t, const vector<double>& x);
    void setOrientation(double x, double y, double theta);
    vector<vector<double>> getObstacleDetections(const sensor_msgs::LaserScan &scan);
    Vector2d getObstacle(int k);
    vector<double> x_state() { return m_vehicle.x_state(); }
    Vector2d vectorFieldControl(int,field,const vector<double>&);
};

Scenario::Scenario(Vehicle &veh)
        :m_vehicle(veh)
{
    m_x_ind = m_vehicle.x_ind();
    m_y_ind = m_vehicle.y_ind();
    m_q_ind << m_x_ind , m_y_ind;

}

Vector2d Scenario::control(int t, const vector<double>& x)
{
    // virtual method
    // return nothing
    return Vector2d(0);
}

void Scenario::setOrientation(double x, double y, double theta)
{
    m_vehicle.setOrientation(x,y,theta);
}

vector<vector<double>> Scenario::getObstacleDetections(const sensor_msgs::LaserScan &scan)
{
    vector<vector<double>> obs;
    obs.push_back(vector<double>(0));
    obs.push_back(vector<double>(0));
    obs.push_back(vector<double>(0));
    return obs;
}

Vector2d Scenario::getObstacle(int k)
{
    return m_vehicle.getObstacle(k);
}

Vector2d Scenario::vectorFieldControl(int t,field f,const vector<double>&x)
{
    m_vehicle.vectorFieldControl(t,f,x);
}