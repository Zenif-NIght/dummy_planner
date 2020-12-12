#include "vectorFollowingTypePoint.hpp"
#include "Vehicle.hpp"
#include "field.hpp"

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>

vectorFollowingTypePoint::vectorFollowingTypePoint()
{
        ROS_INFO("vectorFollowingTypePoing constructor");
}

vectorFollowingTypePoint::~vectorFollowingTypePoint()
{
}

Vector2d vectorFollowingTypePoint::vectorFieldControl(
        Vehicle &veh,
        int t, 
        field &f,
        const vector<double> &x)
{
    // %pointVelocityVectorFieldControl will calculate the desired control to follow 
    // % a vector field with the following inputs (using point control):
    // %   t: Time
    // %   g_function: function handle for obtaining vector.
    // %   g_function is a function of (t, x) and returns the vector
    // %   x: the state
    // %
    // %  With the output
    // %    u: control input to the system (u_v, u_omega)
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // % Get necessary states
    // [v, w] = obj.kinematics.getVelocities(t, x, 0);
    // th = x(obj.th_ind);
    // c = cos(th);
    // s = sin(th);
    ROS_INFO("vfc: field control");
    Vector2d v_w = veh.kinematics().getVelocities(t, x, x);
    ROS_INFO_STREAM("vfc: v_s = "<<v_w);
    double w = v_w[1];
	double th = veh.th();
	double c = cos(th);
	double s = sin(th);
    ROS_INFO_STREAM("vfc: th = "<<th);

    // % Form espilon variables
    // eps = obj.eps_vel;
    // w_hat_e = [0 -eps*w; w/eps 0];
    // R_e = [c -eps*s; s eps*c];
    // R_e_inv = [1 0; 0 1/eps] * [c s; -s c];
    double eps = veh.eps_vel();
	Matrix2d w_hat_e;   w_hat_e << 0, -eps*w, w/eps, 0;
    ROS_INFO_STREAM("vfc: w_hat_e = "<<w_hat_e);
	Matrix2d R_e;       R_e << c, -eps*s, s, eps*c;
	Matrix2d R1;        R1 << 1, 0, 0, 1/eps;
	Matrix2d R2;        R2 << c, s, -s, c;
	Matrix2d R_e_inv = R1 * R2;

    // % Calculate current velocity of espilon state
    // q_eps = x(obj.q_ind) + eps*[c; s];
    // q_eps_dot = R_e*[v; w];
    Vector2d c_s(c, s);
	Vector2d q_eps = veh.q() + c_s*eps;
	Vector2d q_eps_dot = R_e * v_w;
    
    // % Calculate the vector field for the espilon point
    // g = g_function(t, q_eps, th);
    Vector2d g = f.getVector(t, q_eps, th);
    ROS_INFO_STREAM("vfc: g = "<<g);
    
    // % Restrict the velocity of the vector field
    // v_g = norm(g);
    // if v_g > obj.vd_field_max
    //     g = obj.vd_field_max/v_g * g;
    double v_g = g.norm();
    if (v_g > veh.vd_field_max())
    {
        g = veh.vd_field_max()/v_g * g;
    }

    // % Calculate point control
    // u_point = -obj.K_point_vel*(q_eps_dot - g);
    Vector2d u_point; u_point = -veh.K_point_vel() * (q_eps_dot - g);
    ROS_INFO_STREAM("vfc: u_point = "<<u_point);
    
    // % Calculate the commanded acceleration values            
    // u = R_e_inv*u_point - w_hat_e*[v;w]; 
    Vector2d u = R_e_inv*u_point - w_hat_e*v_w;
    ROS_INFO_STREAM("vfc: u = "<<u);

    // return  Vector2d(0);
    return u;

}