#pragma once

//#include "Vehicle.hpp"
//#include "field.hpp"
#include "control_type.hpp"
#include <vector>

class Vehicle;
class field;

using namespace Eigen;
using namespace std;


class vectorFollowingTypePoint : public control_type
{
private:
    
public:
    vectorFollowingTypePoint();
    ~vectorFollowingTypePoint();
    Vector2d vectorFieldControl(
        Vehicle &veh,
        int t, 
        field &f,
        const vector<double> &x);
};

