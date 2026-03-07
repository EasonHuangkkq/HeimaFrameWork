#include "ankle_solver.h"
#include <iostream>
#include <iomanip>
#include <cmath>

int main() {
    AnkleSolver ankle_solver;
    
    std::cout << "=== Testing AnkleSolver ===" << std::endl;
    
    // Test basic coordinates
    double l1 = 69 - 22;
    double d1 = 35;
    
    Eigen::Vector3d coordinate_c(-l1, d1, 0);
    std::cout << "\ncoordinate_c: [" << coordinate_c.transpose() << "]" << std::endl;
    
    Eigen::Vector3d coordinate_c_after_pitch_roll = ankle_solver.get_coord_after_pitch_roll(
        coordinate_c, M_PI / 6.0, 0);
    std::cout << "coordinate_c_after_pitch_roll: [" 
              << coordinate_c_after_pitch_roll.transpose() << "]" << std::endl;
    
    Eigen::Vector3d coordinate_e = ankle_solver.get_E(0);
    std::cout << "coordinate_e: [" << coordinate_e.transpose() << "]" << std::endl;
    
    Eigen::Vector3d coordinate_f = ankle_solver.get_F(0);
    std::cout << "coordinate_f: [" << coordinate_f.transpose() << "]" << std::endl;
    
    // Test forward solve
    std::cout << "\n=== Testing forward solve (pitch, roll -> theta_f, theta_e) ===" << std::endl;
    auto [theta_f, theta_e] = ankle_solver.solve(0.0, 0.6);
    std::cout << "Input: pitch=0.0, roll=0.6" << std::endl;
    std::cout << "Output: theta_f=" << theta_f << ", theta_e=" << theta_e << std::endl;
    
    // Test inverse solve (round-trip verification)
    std::cout << "\n=== Testing inverse solve (round-trip verification) ===" << std::endl;
    double test_pitch = 0.2;
    double test_roll = 0.3;
    
    std::cout << "Forward solve: pitch=" << test_pitch << ", roll=" << test_roll << " -> theta_f, theta_e" << std::endl;
    auto [theta_f_result, theta_e_result] = ankle_solver.solve(test_pitch, test_roll);
    std::cout << "  theta_f=" << theta_f_result << ", theta_e=" << theta_e_result << std::endl;
    
    std::cout << "\nInverse solve: theta_f=" << theta_f_result << ", theta_e=" << theta_e_result << " -> pitch, roll" << std::endl;
    auto [pitch_result, roll_result] = ankle_solver.solve_inverse(theta_e_result, theta_f_result);
    std::cout << "  pitch=" << pitch_result << ", roll=" << roll_result << std::endl;
    
    std::cout << "\n=== Round-trip error ===" << std::endl;
    std::cout << std::fixed << std::setprecision(9);
    std::cout << "Original pitch: " << test_pitch << ", Recovered pitch: " << pitch_result 
              << ", Error: " << std::abs(test_pitch - pitch_result) << std::endl;
    std::cout << "Original roll: " << test_roll << ", Recovered roll: " << roll_result 
              << ", Error: " << std::abs(test_roll - roll_result) << std::endl;
    
    return 0;
}

