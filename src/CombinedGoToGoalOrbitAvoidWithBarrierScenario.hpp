#pragma once

#include "GoToGoalField.hpp"
#include "OrbitAvoidField.hpp"
#include "AvoidObstacle.hpp"
#include "SummedFields.hpp"
#include "VectorFieldScenario.hpp"
#include "Vehicle.hpp"

#include <vector>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class CombinedGoToGoalOrbitAvoidWithBarrierScenario : public VectorFieldScenario
{
private:
    vector<int> m_avoid_indices; // Stores the indices for the avoidance fields
    Vector2d m_q_inf; // Large number to use for placing an obstacle infinitely far away
    int m_n_sensors; // stores the number of sensors
public:
    CombinedGoToGoalOrbitAvoidWithBarrierScenario(Vehicle veh, const Vector2d& x_g);
    ~CombinedGoToGoalOrbitAvoidWithBarrierScenario();
    Vector2d control(int t, const vector<double> &x);
};

CombinedGoToGoalOrbitAvoidWithBarrierScenario::CombinedGoToGoalOrbitAvoidWithBarrierScenario(Vehicle veh, const Vector2d& x_g)
        : VectorFieldScenario(veh)
{
    m_n_sensors = veh.n_sensors();
    ROS_INFO("scenario: n_sensors: %d",m_n_sensors);

    //     % Obstacle avoidance variables - orbit
    double S = 3; //% Sphere of influence
    double R = 2; // Radius of orbit
    double k_conv = 0.5; // Convergence gain
        
    //     % Obstacle avoidance variables - barrier
    double S_b = 1.0; // Sphere of influence of barrier
    double R_b = 0.5; // Radius of full influence
        
    //     % Weights
    double w_g2g = 1;
    double w_avoid = 1;
    double w_barrier = 10;

    // double weights = zeros(1+veh.sensor.n_lines*2, 1);
    vector<double> weights(m_n_sensors * 2 +1,1);
    weights[0] = w_g2g;
    // weights[2:n_lines] = w_avoid;
    // weights[2+n_lines:weights.end()] = w_barrier;
    for (int i=1; i < weights.size(); i++)
    {
        if(i > 1 + m_n_sensors)
            weights[i] = w_barrier;
        else
            weights[i] = w_avoid;
    }
    // % Create a weighted field for the vector field to follow
    // fields = cell(1+veh.sensor.n_lines*2, 1); % 1 for goal to goal and then the rest for the object avoidance
    vector<field> fields(weights.size());
    //     avoid_indices = 2:veh.sensor.n_lines+1;
    m_avoid_indices.resize(m_n_sensors+1);
    for (int i = 0; i < m_n_sensors+1; i++) {m_avoid_indices[i] = i+1;}
    
    // q_inf = [10000000; 10000000];
    m_q_inf << 10000000 , 10000000;

    const double v_max = 2; // max velocity limit

    //     fields{1} = GoToGoalField(x_vec, y_vec, x_g, v_max);
    fields[0] = GoToGoalField( x_g, v_max);
    // for k = avoid_indices
        // fields{k} = OrbitAvoidField(x_vec, y_vec, q_inf, R, v_max, k_conv, S);  
        // fields{k+veh.sensor.n_lines} = AvoidObstacle(x_vec, y_vec, q_inf, v_max);
    //     fields{k+veh.sensor.n_lines}.S = S_b;
    //     fields{k+veh.sensor.n_lines}.R = R_b;
    // end
    for(int k : m_avoid_indices)
    {
        fields[k] = OrbitAvoidField(m_q_inf, R, v_max, k_conv,S);
        fields[k+m_n_sensors] = AvoidObstacle( m_q_inf, v_max, S_b, R_b);
    }
    //ROS_INFO("field[0]: ");


    //     % Create a combined vector field
    //     field = SummedFields(x_vec, y_vec, fields, weights, v_max);
    SummedFields sfield(fields, weights, v_max);
    setVectorField(sfield);
    
}

CombinedGoToGoalOrbitAvoidWithBarrierScenario::~CombinedGoToGoalOrbitAvoidWithBarrierScenario()
{
}

Vector2d CombinedGoToGoalOrbitAvoidWithBarrierScenario::control(int t, const vector<double> &x)
{
    // Get obstacle avoidance readings into the vector fields
    for(int k=0; k < m_n_sensors; k++)
    {
        Vector2d q = getObstacle(k);
        updateField(q,m_avoid_indices[k],m_n_sensors);
    }

    return VectorFieldScenario::control(t,x);
}