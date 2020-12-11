#pragma once

// #include "Vehicle.hpp"
#include "field.hpp"

class Vehicle;

using namespace Eigen;
using namespace std;

class control_type
{
private:
    /* data */
public:
    control_type(/* args */);
    ~control_type();
    Vector2d vectorFieldControl(
        Vehicle veh,
        int t, 
        field f,
        const vector<double> &x);
};

control_type::control_type(/* args */)
{
}

control_type::~control_type()
{
}

Vector2d control_type::vectorFieldControl(
        Vehicle veh,
        int t, 
        field f,
        const vector<double> &x)
{return Vector2d(0,0);}