#include "VehicleKinematics.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class BetterUnicycleKinematics : public VehicleKinematics
{
private:
    int m_v_ind = 3; // translational velocity index
    int m_w_ind = 4; // angular velocity index (omega)
public:
    BetterUnicycleKinematics();
    ~BetterUnicycleKinematics();
    int v_ind() { return m_v_ind; }
    int w_ind() { return m_w_ind; }
    vector<double> kinematics(int t, const vector<double> &x, const vector<double> &u);
    Vector2d getVelocities(int t, const vector<double> &x, const vector<double> &u);
};

BetterUnicycleKinematics::BetterUnicycleKinematics()
    : VehicleKinematics(5)
{
}

BetterUnicycleKinematics::~BetterUnicycleKinematics()
{
}
       
vector<double> BetterUnicycleKinematics::kinematics(int t, const vector<double> &x, const vector<double> &u)
{
    //kinematics Gives the unicycle dynamics
    //   x = [x; y; theta] = [vcos(theta); vsin(theta); omega]
    //   u = [v; omega]
    
    // Extract inputs
    double u_v = u[0]; // Translational velocity
    double u_w = u[1]; // Rotational velocity
    
    // Extract states
    double v = x[m_v_ind];
    double w = x[m_w_ind];
    
    // Calculate dynamics
    double theta = x[th_ind()];  // Orientation
    vector<double> xdot(dimensions(),0);
    xdot[x_ind()] = v * cos(theta); // \dot{x}
    xdot[y_ind()] = v * sin(theta); // \dot{y}
    xdot[th_ind()] = w; // \dot{theta}            
    xdot[m_v_ind] = u_v;
    xdot[m_w_ind] = u_w;

    return xdot;
}   
        
Vector2d BetterUnicycleKinematics::getVelocities(int t, const vector<double> &x, const vector<double> &u)
{
    Vector2d vel(x[m_v_ind],x[m_w_ind]);
    return vel;
}
