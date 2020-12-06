# pragma once

#include <vector>
#include "Sensor.hpp"

class RangeSensor : public Sensor
{
// RangeSensor Creates a sensor based on a limited range
private:
    int m_n_lines = 10; // number of range measurements
    int m_max_dist = 4; // max distance of the range measurements
    std::vector<double> m_orien_nom; // stores the nominal orientation of each of the range lines
    std::vector<int> m_ind_left; // indices for sensors on the left of the vehicle
    int m_n_left = 0; // number of left sensors
    std::vector<int> m_ind_right; // indices for sensors on the right of the vehicle
    int m_n_right = 0; // number of right sensors
    std::vector<int> m_ind_front; // indices for sensors on the front of the vehicle
    int m_n_front = 0; // number of front sensors
    std::vector<int> m_ind_front_left; // indices for sensors on the front left of the vehicle
    int m_n_front_left = 0; // number of front left sensors
    std::vector<int> m_ind_front_right; // indices for sensors on the front right of the vehicle
    int m_n_front_right = 0; // number of front right sensors


public:
    RangeSensor(int n_lines = 0, int max_dist = 0);
    void InitializeSensor(int n_lines; int max_dist);
}

RangeSensor::RangeSensor(int n_lines, int max_dist)
{
    m_n_lines = n_lines;
    m_max_dist = max_dist;
    InitializeSensor(m_n_lines, m_max_dist);
}

void RangeSensor::InitializeSensor(int n_lines, int max_dist)
{
    m_n_lines = n_lines;
    m_max_dist = max_dist;

    //Create the nominal orientations  of the range lines
    if (m_n_lines <= 1) m_orien_nom = 0; // single line sensor directly out front
    else {
        orien
    }

}