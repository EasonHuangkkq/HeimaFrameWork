#include "ankle_solver.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main() {
    double h1 = 358.74 - (-25.06);
    double h2 = 263.51 - (-25.06);
    
    double m = 316.55;
    double n = 232.27;
    
    double r_E = std::sqrt(std::pow(-69 - (-22), 2) + std::pow(263.51 - 232.27, 2));
    double r_F = std::sqrt(std::pow(-69 - (-22), 2) + std::pow(358.74 - 316.55, 2));
    double l1 = 69 - 22;
    
    double d1 = 35;
    
    AnkleSolver ankle_solver(l1, d1, m, n, h1, h2, r_E, r_F);
    
    Eigen::Vector3d coordinate_c(-l1, d1, 0);
    std::cout << "coordinate_c: [" << coordinate_c.transpose() << "]" << std::endl;
    
    Eigen::Vector3d coordinate_c_after_pitch_roll = ankle_solver.get_coord_after_pitch_roll(
        coordinate_c, M_PI / 6.0, 0);
    std::cout << "coordinate_c_after_pitch_roll: [" 
              << coordinate_c_after_pitch_roll.transpose() << "]" << std::endl;
    
    Eigen::Vector3d coordinate_e = ankle_solver.get_E(0);
    std::cout << "coordinate_e: [" << coordinate_e.transpose() << "]" << std::endl;
    
    Eigen::Vector3d coordinate_f = ankle_solver.get_F(0);
    std::cout << "coordinate_f: [" << coordinate_f.transpose() << "]" << std::endl;
    
    auto [theta_f, theta_e] = ankle_solver.solve(0, 0);
    std::cout << "theta_f: " << theta_f << ", theta_e: " << theta_e << std::endl;
    
    // Draw/get CE values for plotting (equivalent to Python's linspace)
    std::cout << "\nCE values for theta_e from -pi to pi:" << std::endl;
    const int num_points = 100;
    for (int i = 0; i < num_points; ++i) {
        double theta = -M_PI + (2.0 * M_PI * i) / (num_points - 1);
        double ce_value = ankle_solver.get_CE(0, 0, theta);
        std::cout << std::fixed << std::setprecision(6) 
                  << theta << " " << ce_value << std::endl;
    }
    
    return 0;
}

