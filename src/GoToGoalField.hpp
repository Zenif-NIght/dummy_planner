#pragma once

#include "field.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>

class GoToGoalField : public field
{
// GoToGoalField Basic vector field pointing to a desired goal point
private:
    Eigen::Vector2d m_x_g; //% Goal position
    double m_v_max; //% Max velocity
    
    // % Convergence variables
    double m_sig = 1; //% Effects the convergence to zero velocity 1-exp(-d^2/sig^2) through 1-exp(-d^2/sig^2)
    double m_sig_sq; //% Sig^2
public:
    GoToGoalField(const Eigen::Vector2d & x_g, double v_max);
    ~GoToGoalField();
    Eigen::Vector2d getVector(int t, const Eigen::Vector2d& x, double th);
};

//Constructor 
GoToGoalField::GoToGoalField(const Eigen::Vector2d & x_g, double v_max)
{            
    // % Create the object variable
    // obj = obj@VectorField(x_vec, y_vec);
    m_x_g = x_g;
    m_v_max = v_max;

    // % Initialize scaling variable
    m_sig_sq = m_sig*m_sig;
}

GoToGoalField::~GoToGoalField()
{
}

Eigen::Vector2d GoToGoalField::getVector(int t, const Eigen::Vector2d& x, double th)
{
    // getvector will return a go-to-goal vector given the position x

    // Calculate the diffence vector from vehicle position to goal
    Eigen::Vector2d g = m_x_g - x;
            
    // Scale the magnitude of the resulting vector using a smooth
    // convergence
    double dist = g.norm();
    double v_g = m_v_max * (1- exp(-pow(dist,2)/m_sig_sq));
    
    // Check distance prior to dividing by zero
    if (dist > 0) // Avoid dividing by zero
    {
        g = v_g/dist * g; // Dividing by dist is dividing by the norm
    }
    else g << 0,0;

    return g;
}