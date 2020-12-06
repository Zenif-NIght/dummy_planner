#include "field.hpp"
//#include <eql/linear_algebra.hpp>
#include <cmath>

class OrbitAvoidField : public field
{
private:
    Eigen::Vector2d m_x_o; // 2D center position of the orbit (typically updated to use the obstacle position as the center of the orbit)
    double m_rad; // radius of the orbit
    double m_w; // frequency of the orbit (calculated using v = wr relationship)
    double m_k_conv; // gain on convergence to the orbit
    double m_v_d; // Desired speed of the orbit 

    double m_S; // Sphere of influence
public:
    OrbitAvoidField(const Eigen::Vector2d &x_o,double rad,double v_d, double k_conv,double S);
    ~OrbitAvoidField();
    Eigen::Vector2d getVector(int t, const Eigen::Vector2d& x, double th);
};

OrbitAvoidField::OrbitAvoidField( const Eigen::Vector2d &x_o,
                                    double rad,
                                    double v_d, 
                                    double k_conv,
                                    double S)
{
    // Store the variables which describe the orbit
    m_x_o = x_o;
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

Eigen::Vector2d OrbitAvoidField::getVector(int t, const Eigen::Vector2d& x, double th)
{
    // getVector will return a go-to-goal vector given the position x
    //
    // Inputs:
    //   x: 2D position for calculating the vector
    //   th: Current orientation of the vehicle

    // Calculate the difference between the vector and obstacle
    Eigen::Vector2d xhat = (x - m_x_o);
    
    // Calculate the orientation difference from direction towards
    // obstacle
    double th_e =  th - atan2(xhat(0),xhat(1))  ;
    
    // Calculate the rotation based on the angle to the obstacle
    double w_act = abs(m_w);
    if (th_e > 0) // th_e is the difference between the vehicle orientation and the obstacle
    {
        w_act = -w_act;
    }
    
    // Get the effective radius of the vecicle from the center
    double rad_e = m_v_d/w_act;
    
    // Calculate convergence gain
        // Don't attract vehicle to the orbit
        double gam = m_k_conv*(pow(rad_e,2) - (xhat.transpose()).dot(xhat));
                
    // Create the orbit vector
    Eigen::Matrix2d A;
    A << gam, w_act, -w_act, gam;  
    Eigen::Vector2d g = A * xhat; 
    
    // Scale the vector field based on the sphere of influence
    double d = g.norm(); // TODO might need to look in to this 
    int k_i = 0;
    if (d > m_S) k_i = 0;
    else if (rad_e < d && d <= m_S) k_i = (m_S - d)/(m_S - rad_e);
    else if (d <= rad_e) k_i = 0;
    else k_i = -10000000; // "k_i ERROR";
    
    g = g * k_i;
    
    // Scale the vector field by the orientation influence
    int k_o =0;
    if (abs(th) <= M_PI/2) k_o = 1;
    else k_o = 0;
    
    g = g * k_o;
    
    // Threshold the vector field to be less than or equal to the
    // desired velocity
    
    
    return g;
}