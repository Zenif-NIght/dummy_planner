#include "control_type.hpp"
#include "Vehicle.hpp"

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