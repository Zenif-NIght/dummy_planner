#pragma once

#include "GoToGoalField.hpp"
#include "OrbitAvoidField.hpp"
#include "AvoidObstacleField.hpp"
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
    // aligned?? Vector2d m_q_inf; // Large number to use for placing an obstacle infinitely far away
    int m_n_sensors; // stores the number of sensors
public:
    CombinedGoToGoalOrbitAvoidWithBarrierScenario(Vehicle &veh, const Vector2d& x_g);
    ~CombinedGoToGoalOrbitAvoidWithBarrierScenario();
    Vector2d control(int t, const vector<double> &x);
};

CombinedGoToGoalOrbitAvoidWithBarrierScenario::CombinedGoToGoalOrbitAvoidWithBarrierScenario(Vehicle &veh, const Vector2d& x_g)
        : VectorFieldScenario(veh)
{
    m_n_sensors = veh.n_sensors();
    ROS_INFO_STREAM("Combined: n_sensors: "<<m_n_sensors);

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
    ROS_INFO("  weights");
    vector<double> weights(m_n_sensors * 2 +1,1);
    ROS_INFO_STREAM("  size = "<<weights.size());
    weights[0] = w_g2g;
    // weights[2:n_lines] = w_avoid;
    // weights[2+n_lines:weights.end()] = w_barrier;
    for (int i=1; i < weights.size(); i++)
    {
        ROS_INFO("  init weignt %i",i);
        if(i > 1 + m_n_sensors)
            weights[i] = w_barrier;
        else
            weights[i] = w_avoid;
    }
    ROS_INFO("Create a vector of fields");
    // % Create a weighted field for the vector field to follow
    // fields = cell(1+veh.sensor.n_lines*2, 1); % 1 for goal to goal and then the rest for the object avoidance
    vector<field> fields(weights.size());
    //     avoid_indices = 2:veh.sensor.n_lines+1;
    m_avoid_indices.resize(m_n_sensors);
    for (int i = 0; i < m_n_sensors+1; i++) {m_avoid_indices[i] = i+1;}
    ROS_INFO_STREAM("Combined: avoid_indices: size: "<<m_avoid_indices.size());
    //ROS_INFO_STREAM("   [] = " <<m_avoid_indices);

    // q_inf = [10000000; 10000000];
    Vector2d q_inf( 10000000 , 10000000); // infinity

    const double v_max = 2; // max velocity limit

    //     fields{1} = GoToGoalField(x_vec, y_vec, x_g, v_max);
    fields[0] = GoToGoalField( x_g, v_max);
    // for k = avoid_indices
        // fields{k} = OrbitAvoidField(x_vec, y_vec, q_inf, R, v_max, k_conv, S);  
        // fields{k+veh.sensor.n_lines} = AvoidObstacleField(x_vec, y_vec, q_inf, v_max);
    //     fields{k+veh.sensor.n_lines}.S = S_b;
    //     fields{k+veh.sensor.n_lines}.R = R_b;
    // end
    ROS_INFO_STREAM("Combined: set field objects for "<<m_n_sensors);
    for(int k : m_avoid_indices)
    {
        ROS_INFO("  set field %d",k);
        fields[k] = OrbitAvoidField(q_inf, R, v_max, k_conv,S);
        fields[k+m_n_sensors] = AvoidObstacleField( q_inf, v_max, S_b, R_b);
    }
    //ROS_INFO("field[0]: ");


    //     % Create a combined vector field
    //     field = SummedFields(x_vec, y_vec, fields, weights, v_max);
    ROS_INFO("sum fields...");
    SummedFields sfield(fields, weights, v_max);
    ROS_INFO("setVectorFIeld");
    setVectorField(sfield);
    ROS_INFO("construction done");
}

CombinedGoToGoalOrbitAvoidWithBarrierScenario::~CombinedGoToGoalOrbitAvoidWithBarrierScenario()
{
}

Vector2d CombinedGoToGoalOrbitAvoidWithBarrierScenario::control(int t, const vector<double> &x)
{
    ROS_INFO("Combine control entry; update fields");
    // Get obstacle avoidance readings into the vector fields
    for(int k=0; k < m_n_sensors; k++)
    {
        Vector2d q = getObstacle(k);
        ROS_INFO_STREAM("Obstacle "<<k<<" at "<<q);
        updateField(q,m_avoid_indices[k],m_n_sensors);
    }

    ROS_INFO("get fector field control variables");
    return VectorFieldScenario::control(t,x);
}