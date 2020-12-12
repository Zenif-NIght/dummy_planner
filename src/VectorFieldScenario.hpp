#pragma once

#include "Scenario.hpp"
#include "field.hpp"
#include "Vehicle.hpp"

using namespace Eigen;
using namespace std;

class VectorFieldScenario : public Scenario
{
// VectorFieldScenario controls a vehicle to follow a given vector field
private:
    field m_vector_field;
    // control_type; // the type of vector field control to be used
public:
    VectorFieldScenario(Vehicle &veh);
    ~VectorFieldScenario();
    void setVectorField(field f);
    void updateField(const Vector2d &q,int i,int n_lines);
    Vector2d control(int t, const vector<double> &x);
};

VectorFieldScenario::VectorFieldScenario(Vehicle &veh)
        : Scenario(veh)
{
}

VectorFieldScenario::~VectorFieldScenario()
{
}

void VectorFieldScenario::setVectorField(field f)
{
    m_vector_field = f;
}

void VectorFieldScenario::updateField(const Vector2d &q,int i,int n_lines)
{
    m_vector_field.updateField(q,i,n_lines);
}

Vector2d VectorFieldScenario::control(int t, const vector<double> &x)
{
    ROS_INFO("VectorFieldScenario control");
    Vector2d u = vectorFieldControl(t,m_vector_field,x);
    return u;
}