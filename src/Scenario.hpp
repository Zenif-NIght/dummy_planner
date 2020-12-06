#include <eigen3/Eigen/Dense>

#pragma once

class Scenario
{
private:
    // Define elements to be simulated
    Vechicle m_vehicle; // instance of the Vehicle class
    World m_world; // instance of the Polygon world class

    // Simulation parameters
    int m_t_0; // initial time of simulation
    double m_dt = 0.05; // Simulation step size
    int m_tf = 25; // Final time of simulation

    // index variables
    int m_x_ind; // x position index
    int m_y_ind; // y position index
    Eigen::Vector2d m_q_ind; // 2D position index

public:
    Scenario(Vehicle veh, World world);
}

Scenario::Scenario(Vehicle veh, World world)
{
    m_vehicle = veh;
    m_world = world;

    m_x_ind = m_vehicle.x_ind();
    m_y_ind = m_vehicle.y_ind();
    m_q_ind << m_x_ind , m_y_ind;

    m_vehicle.getObstacleDetections(m_world);
}