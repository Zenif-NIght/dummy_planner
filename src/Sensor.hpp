#pragma once

#include <vector>

class Sensor
{
public:
    Sensor() {}
    virtual int n_lines() { return 0; }
    virtual std::vector<std::vector<double>> getObstacleDetections(const Eigen::Vector2d &q, double th, const sensor_msgs::LaserScan &scan);
};

std::vector<std::vector<double>> Sensor::getObstacleDetections(const Eigen::Vector2d &q, double th, const sensor_msgs::LaserScan &scan)
{
    std::vector<std::vector<double>> obs;
    obs.push_back(std::vector<double>(0));
    obs.push_back(std::vector<double>(0));
    obs.push_back(std::vector<double>(0));
    return obs;
}