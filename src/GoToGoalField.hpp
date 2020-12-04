#include "field.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
class GoToGoalField : public field
{
private:
    double m_x_g[2]; //% Goal position
    double m_v_max; //% Max velocity
    
    // % Convergence variables
    double m_sig = 1; //% Effects the convergence to zero velocity through 1-exp(-d^2/sig^2)
    double m_sig_sq; //% Sig^2
public:
    GoToGoalField(geometry_msgs::Point x_g, double v_max);
    ~GoToGoalField();
};

//Constructor 
GoToGoalField::GoToGoalField( geometry_msgs::Point x_g, double v_max)
{            
    // % Create the object variable
    // obj = obj@VectorField(x_vec, y_vec);
    m_x_g[0] = x_g.x;
    m_x_g[1] = x_g.y;
    m_v_max = v_max;

    // % Initialize scaling variable
    m_sig_sq = m_sig*m_sig;
}

GoToGoalField::~GoToGoalField()
{
}