#include "field.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
class OrbitAvoidField : public field
{
private:
    geometry_msgs::Point m_x_o; // 2D center position of the orbit (typically updated to use the obstacle position as the center of the orbit)
    double m_rad; // radius of the orbit
    double m_w; // frequency of the orbit (calculated using v = wr relationship)
    double m_k_conv; // gain on convergence to the orbit
    double m_v_d; // Desired speed of the orbit 

    double m_S; // Sphere of influence
public:
    OrbitAvoidField(const geometry_msgs::Point &x_o,double rad,double v_d, double k_conv,double S);
    ~OrbitAvoidField();
};

OrbitAvoidField::OrbitAvoidField( const geometry_msgs::Point &x_o,
                                    double rad,
                                    double v_d, 
                                    double k_conv,
                                    double S)
{
    // Store the variables which describe the orbit
    m_x_o.x = x_o.x;
    m_x_o.y= x_o.y;
    m_rad = rad;
    m_k_conv = k_conv;
    m_S = S;

    // Calculate the rotational velocity
    m_v_d = abs(v_d);
    m_w = m_v_d/m_rad;
}

OrbitAvoidField::~OrbitAvoidField()
{
}
