#include "GoToGoalField.hpp"
#include "OrbitAvoidField.hpp"
#include "AvoidObstacle.hpp"

#include <eigen3/Eigen/Dense>

class CombinedGoToGoalOrbitAvoidWithBarrierScenario
{
private:
    /* data */
public:
    CombinedGoToGoalOrbitAvoidWithBarrierScenario(/* args */);
    ~CombinedGoToGoalOrbitAvoidWithBarrierScenario();
};

CombinedGoToGoalOrbitAvoidWithBarrierScenario::CombinedGoToGoalOrbitAvoidWithBarrierScenario(/* args */)
{
        //         % Go to goal variables
    //     x_g =  [20; 5]; %[16; 10]; % try [20; 5]; [25; 5];
    // geometry_msgs::Point x_g;
    // x_g.x = result.pose.position.x;
    // x_g.y = result.pose.position.y;
    Eigen::Vector2d x_g;
    // x_g << result.pose.position.x , result.pose.position.y;
        
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
    // if(!scan)
    //     return;

    int n_lines = 12;//scan->ranges.size() ;
    std::vector<double> weights(n_lines * 2 +1,1);
    weights[0] = w_g2g;
    // weights[2:n_lines] = w_avoid;
    // weights[2+n_lines:weights.end()] = w_barrier;
    for (int i=1; i < weights.size(); i++)
    {
        if(i > 1+n_lines)
            weights[i] = w_barrier;
        else
            weights[i] = w_avoid;
    }
    // % Create a weighted field for the vector field to follow
    // fields = cell(1+veh.sensor.n_lines*2, 1); % 1 for goal to goal and then the rest for the object avoidance
    std::vector<field> fields(weights.size());
    //     avoid_indices = 2:veh.sensor.n_lines+1;
    std::vector<int> avoid_indices(n_lines+1);
    for (int i = 0; i < n_lines+1; i++) {avoid_indices[i] = i+1;}
    
    // q_inf = [10000000; 10000000];
    Eigen::Vector2d q_inf;
    q_inf << 10000000 , 10000000;

    const double v_max = 2;
    //     fields{1} = GoToGoalField(x_vec, y_vec, x_g, v_max);
    fields[0] = GoToGoalField( x_g, v_max);
    // for k = avoid_indices
        // fields{k} = OrbitAvoidField(x_vec, y_vec, q_inf, R, v_max, k_conv, S);  
        // fields{k+veh.sensor.n_lines} = AvoidObstacle(x_vec, y_vec, q_inf, v_max);
    //     fields{k+veh.sensor.n_lines}.S = S_b;
    //     fields{k+veh.sensor.n_lines}.R = R_b;
    // end
    for (int i : avoid_indices)
    {
        fields[i] = OrbitAvoidField(q_inf, R, v_max, k_conv,S);
        fields[i+n_lines] = AvoidObstacle( q_inf, v_max, S_b, R_b);
    }  


    //     % Create a combined vector field
    //     field = SummedFields(x_vec, y_vec, fields, weights, v_max);
    field sfield = SummedFields(fields, weights, v_max);
    
    // //     % Create the scenario
    // //     obj = obj@VectorFieldScenario(field, veh, PolygonWorld1, control_type);
        
    // // %             veh.x(veh.q_ind) = [5; -1];
    // // %             veh.x(veh.th_ind) = pi/4;
        
    // //     % Store object variables
    // //     obj.avoid_indices = avoid_indices;
    // //     obj.q_inf = q_inf; 
    // //     obj.n_sensors = veh.sensor.n_lines;
}

CombinedGoToGoalOrbitAvoidWithBarrierScenario::~CombinedGoToGoalOrbitAvoidWithBarrierScenario()
{
}
