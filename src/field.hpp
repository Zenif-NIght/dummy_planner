#pragma once
// #ifndef FIELD_DEFIND

#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class field
{
// VectorField is an abstract class which gives a basic vector field for navigation
private:

public:
    // double S;
    // double R;
    field();
    ~field() {}
    void set_x_o(const Vector2d &v) {}
    void updateField(const Vector2d &q,int i,int n_lines) {}
    // Abstract
    virtual Vector2d getVector(int t, const Vector2d& x, double th) {return Vector2d(0);}
};

field::field()
{
}