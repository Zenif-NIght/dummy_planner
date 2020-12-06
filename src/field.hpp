#pragma once
// #ifndef FIELD_DEFIND

#include <eigen3/Eigen/Dense>

class field
{
// VectorField is an abstract class which gives a basic vector field for navigation
private:

public:
    // double S;
    // double R;
    field();
    ~field() {}
    // Abstract
    virtual Eigen::Vector2d getVector(int t, const Eigen::Vector2d& x, double th);
};

field::field()
{
}

Eigen::Vector2d field::getVector(int t, const Eigen::Vector2d& x, double th)
{
    Eigen::Vector2d g;
    g << 0,0;
    return g;
}