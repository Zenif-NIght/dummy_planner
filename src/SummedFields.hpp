#include "field.hpp"

class SummedFields : public field
{
// SummedFields sums together multiple fields to get the contribution
private:
    std::vector<field> m_fields; // structure of field objects
    int m_n_fields; // length of m_fields
    std::vector<double> m_weights; // vector, length m_fields, which provides a weight for each field
    double m_v_max; // maximum velocity produced by the field

public:
    SummedFields(const std::vector<field>& fields, const std::vector<double>& weights, double v_max);
    Eigen::Vector2d getVector(int t, const Eigen::Vector2d& x, double th);
};

SummedFields::SummedFields(const std::vector<field>& fields, const std::vector<double>& weights, double v_max)
{
    m_fields = fields;
    m_n_fields = fields.size();
    m_weights = weights;
    m_v_max = v_max;
}

Eigen::Vector2d SummedFields::getVector(int t, const Eigen::Vector2d& x, double th)
{
    Eigen::Vector2d g;
    g << 0, 0;

    for (int k=0; k<m_n_fields; k++) 
    {
        g = g + m_weights[k] * m_fields[k].getVector(t,x,th);
    }

    // Saturate the field to have a maximum velocity of v_max
    double v_g = g.norm();
    if (v_g > m_v_max) g = m_v_max/v_g * g;

    return g;
}