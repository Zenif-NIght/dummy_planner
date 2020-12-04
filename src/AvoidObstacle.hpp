#include "field.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
class AvoidObstacle : public field
{
private:
geometry_msgs::Point m_x_o ; // Obsactle position
double m_v_max ; // Max velocity
        
// Convergence variables
double m_S = 5 ; // Sphere of influence
double m_R = 1 ; // Radius of max effect 
public:
    AvoidObstacle(const geometry_msgs::Point &x_o,double vmax);
    ~AvoidObstacle();
};

AvoidObstacle::AvoidObstacle(const geometry_msgs::Point &x_o,double v_max)
{
    m_x_o.x = x_o.x;
    m_x_o.y= x_o.y;
    m_v_max = v_max;
}
AvoidObstacle::~AvoidObstacle()
{
}
