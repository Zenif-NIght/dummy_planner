# pragma once

#include <sensor_msgs/LaserScan.h>
#include <vector>
#include "Sensor.hpp"

class RangeSensor : public Sensor
{
// RangeSensor Creates a sensor based on a limited range
private:
    int m_n_lines; // number of range measurements
    int m_max_dist; // max distance of the range measurements
    int m_min_dist; // min distance of the range measurements
    double m_angle_min; // min sensor angle
    double m_angle_max; // max sensor angle
    double m_angle_inc; // angle increment between sensors
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
    RangeSensor(const sensor_msgs::LaserScan &scan);
    void InitializeSensor(const sensor_msgs::LaserScan &scan);
    virtual int n_lines() { return m_n_lines; }
    std::vector<std::vector<double>> getObstacleDetections(const Eigen::Vector2d &q, double th, const sensor_msgs::LaserScan &scan);
};

RangeSensor::RangeSensor(const sensor_msgs::LaserScan &scan)
        : Sensor()
{
    InitializeSensor(scan);
}

void RangeSensor::InitializeSensor(const sensor_msgs::LaserScan &scan)
{
    m_n_lines = (scan.angle_max - scan.angle_min)/scan.angle_increment + 1;
    m_max_dist = scan.range_max;
    m_min_dist = scan.range_min;
    m_angle_min = scan.angle_min;
    m_angle_max = scan.angle_max;
    m_angle_inc = scan.angle_increment;

    ROS_INFO("scan lines: %d",m_n_lines);
    ROS_INFO("scan angles: %f to %f at %f increments",scan.angle_min, scan.angle_max, scan.angle_increment);

    bool front;
    double a = scan.angle_min;
    for (int k = 0; a <= scan.angle_max; a+=scan.angle_increment, k++)
    {
        double th = a;
        th = atan2(sin(th),cos(th));
        m_orien_nom.push_back(th); // adjust to be between -pi and pi

        // Add it to the front if the angle is less than pi/2
        if (abs(th) < M_PI/2)
        {
            m_ind_front.push_back(k);
            front = true;
        }
        else front = false;

        // Add it to the left if orientation is positive
        if (th > 0)
        {
            m_ind_left.push_back(k);
            if (front) m_ind_front_left.push_back(k);
        }
        else // otherwise consider it on the right of the vehicle
        {
            m_ind_right.push_back(k);
            if (front) m_ind_front_right.push_back(k);
        }
    }
}

std::vector<std::vector<double>> RangeSensor::getObstacleDetections(const Eigen::Vector2d &q, double th, const sensor_msgs::LaserScan &scan)
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> d;

    int i =0;
    for(double a = m_angle_min; a <= m_angle_max; a += m_angle_inc, i++)
    {
        double c = cos(a+th);
        double s = sin(a+th);
        double dist = scan.ranges[i];
        x.push_back(q.x() + dist * c);
        y.push_back(q.y() + dist * s);
        d.push_back(dist);
    }

    std::vector<std::vector<double>> obs;
    obs.push_back(x);
    obs.push_back(y);
    obs.push_back(d);

    return obs;
}