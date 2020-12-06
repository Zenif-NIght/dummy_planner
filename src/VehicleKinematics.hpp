#pragma once

class VehicleKinematics
{
// VehicleKinematics Abstract class defining the required methods for a dynamics class

private:
    // State variables
    int m_x_ind = 0; // index into sate of x position
    int m_y_ind = 1; // index into state of y osition
    int m_th_ind = 2; // index into state of orientation
    int m_dimensions = 3; // number of dimensions (size) of state
    int m_n_u; // number of control inputs

public:
    VehicleKinematics(int dim);
    int getNumberControlInputs();
    // Abstract methods
    std::vector<double> kinematics(int t, const std::vector<double>& x, const std::vector<double>& u) = 0;
    std::vector<double> getVelocities(int t, const std::vector<double>& x, const std::vector<double>& u) = 0;
}

VehicleKinematics::VehicleKinematics(int dem)
{
    m_dimensions = dem;
    m_n_u = getNumberControlInputs();
}

int VehicleKinematics::getNumberControlInputs()
{
    // default is to return 2 control inputs
    return 2;
}
