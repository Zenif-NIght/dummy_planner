#pragma once

#include <vector>
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
    control_type() {}
    ~control_type() {}
    
    //Pure Virtual methods
    virtual Vector2d vectorFieldControl(
        Vehicle &veh,
        int t, 
        field &f,
        const vector<double> &x) = 0;
};