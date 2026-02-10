#ifndef ANKLE_SOLVER_H
#define ANKLE_SOLVER_H

#include <Eigen/Dense>
#include <cmath>
#include <functional>

class AnkleSolver {
public:
    AnkleSolver();
    
    // Solve for theta_f and theta_e given pitch and roll
    std::pair<double, double> solve(double pitch, double roll);
    
    // Inverse solve: compute pitch and roll given theta_e and theta_f
    std::pair<double, double> solve_inverse(double theta_e, double theta_f);
    
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
    
    // Calculate distance between two 3D coordinates
    double get_distance(const Eigen::Vector3d& coord1, const Eigen::Vector3d& coord2);
    
    // Apply rotation around X axis
    Eigen::Vector3d apply_rotation_x(const Eigen::Vector3d& coordinate, double angle);
    
    // Apply rotation around Y axis
    Eigen::Vector3d apply_rotation_y(const Eigen::Vector3d& coordinate, double angle);
    
    // Root finding using Newton-Raphson method (1D)
    double fsolve(std::function<double(double)> func, double initial_guess);
    
    // Root finding using Newton-Raphson method (2D)
    std::pair<double, double> fsolve_2d(
        std::function<Eigen::Vector2d(const Eigen::Vector2d&)> func,
        const Eigen::Vector2d& initial_guess);
};

#endif // ANKLE_SOLVER_H

