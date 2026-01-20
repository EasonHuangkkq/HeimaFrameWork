#ifndef ANKLE_SOLVER_H
#define ANKLE_SOLVER_H

#include <Eigen/Dense>
#include <cmath>
#include <functional>

class AnkleSolver {
public:
    AnkleSolver(double l1, double d1, double m, double n, 
                double h1, double h2, double r_E, double r_F);
    
    // Solve for theta_f and theta_e given pitch and roll
    std::pair<double, double> solve(double pitch, double roll);
    
    // Get distance between D and F after rotation
    double get_DF(double pitch, double roll, double theta_f);
    
    // Get distance between C and E after rotation
    double get_CE(double pitch, double roll, double theta_e);
    
    // Get coordinate E
    Eigen::Vector3d get_E(double theta);
    
    // Get coordinate F
    Eigen::Vector3d get_F(double theta);
    
    // Get coordinate D after rotation
    Eigen::Vector3d get_D_after_rotation(double pitch, double roll);
    
    // Get coordinate C after rotation
    Eigen::Vector3d get_C_after_rotation(double pitch, double roll);
    
    // Apply pitch and roll rotation to a coordinate
    Eigen::Vector3d get_coord_after_pitch_roll(const Eigen::Vector3d& coordinate, 
                                                 double pitch, double roll);

private:
    double l1, d1, m, n, h1, h2, r_E, r_F;
    
    // Calculate distance between two 3D coordinates
    double get_distance(const Eigen::Vector3d& coord1, const Eigen::Vector3d& coord2);
    
    // Apply rotation around X axis
    Eigen::Vector3d apply_rotation_x(const Eigen::Vector3d& coordinate, double angle);
    
    // Apply rotation around Y axis
    Eigen::Vector3d apply_rotation_y(const Eigen::Vector3d& coordinate, double angle);
    
    // Root finding using Newton-Raphson method
    double fsolve(std::function<double(double)> func, double initial_guess);
};

#endif // ANKLE_SOLVER_H

