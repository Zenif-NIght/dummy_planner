#pragma once

#include "field.hpp"

#include <eigen3/Eigen/Dense>

class AvoidObstacleField : public field
{
private:
    Eigen::Vector2d m_x_o ; // Obstacle position
    double m_v_max ; // Max velocity

    // Convergence variables
    double m_S; // Sphere of influence
    double m_R; // Radius of max effect 
public:
    AvoidObstacleField(const Eigen::Vector2d &x_o,double vmax, double S=5, double R=1);
    ~AvoidObstacleField();
    void set_x_o(const Eigen::Vector2d &q) { m_x_o = q; }
    Eigen::Vector2d getVector(int t, const Eigen::Vector2d& x, double th);
};

AvoidObstacleField::AvoidObstacleField(const Eigen::Vector2d &x_o,double v_max, double S, double R)
{
    m_x_o = x_o;
    m_v_max = v_max;
    m_S = S;
    m_R = R;
}
AvoidObstacleField::~AvoidObstacleField()
{
}

Eigen::Vector2d AvoidObstacleField::getVector(int t, const Eigen::Vector2d& x, double th)
{
    // Calculate obstacle avoidance vector
    
    // Calculate the difference vector from vehicle position to goal
    Eigen::Vector2d g = -(m_x_o - x);

    // Scale the magnitude of the resulting vector using a smooth
    // convergence
    double dist = g.norm();
    if (m_S - m_R == 0)
    {
        g << 0,0;
        return g;
    }

    double v_g = m_v_max * (m_S-dist)/(m_S-m_R); // exp(-dist^2/S)

    // Check distance prior to dividing by zero
    if (dist > 0)
    {
        if (dist > m_S) g << 0,0;
        else if (dist <= m_S && m_S >= m_R) // avoid dividing by zero
        {
            g = v_g /dist * g; // Dividing by dist is dividing by the norm
        }
        else if (dist > 0 && dist < m_R)
        {
            g = m_v_max/dist * g;
        }
    }
    else g << 0,0;

    return g;
}