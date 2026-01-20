#include "ankle_solver.h"
#include <iostream>
#include <iomanip>

AnkleSolver::AnkleSolver(double l1, double d1, double m, double n, 
                         double h1, double h2, double r_E, double r_F)
    : l1(l1), d1(d1), m(m), n(n), h1(h1), h2(h2), r_E(r_E), r_F(r_F) {
}

double AnkleSolver::get_distance(const Eigen::Vector3d& coord1, const Eigen::Vector3d& coord2) {
    return (coord1 - coord2).norm();
}

Eigen::Vector3d AnkleSolver::apply_rotation_x(const Eigen::Vector3d& coordinate, double angle) {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 1, 0, 0,
                       0, std::cos(angle), -std::sin(angle),
                       0, std::sin(angle), std::cos(angle);
    return rotation_matrix * coordinate;
}

Eigen::Vector3d AnkleSolver::apply_rotation_y(const Eigen::Vector3d& coordinate, double angle) {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << std::cos(angle), 0, std::sin(angle),
                       0, 1, 0,
                       -std::sin(angle), 0, std::cos(angle);
    return rotation_matrix * coordinate;
}

Eigen::Vector3d AnkleSolver::get_coord_after_pitch_roll(const Eigen::Vector3d& coordinate, 
                                                         double pitch, double roll) {
    Eigen::Vector3d result = apply_rotation_x(coordinate, roll);
    result = apply_rotation_y(result, pitch);
    return result;
}

Eigen::Vector3d AnkleSolver::get_D_after_rotation(double pitch, double roll) {
    Eigen::Vector3d temp_d(-69, -35, -25.06);
    return get_coord_after_pitch_roll(temp_d, pitch, roll);
}

Eigen::Vector3d AnkleSolver::get_C_after_rotation(double pitch, double roll) {
    Eigen::Vector3d temp_c(-69, 35, -25.06);
    return get_coord_after_pitch_roll(temp_c, pitch, roll);
}

Eigen::Vector3d AnkleSolver::get_E(double theta) {
    Eigen::Vector3d temp(-r_E, d1, 0);
    temp = apply_rotation_y(temp, theta);
    temp = temp + Eigen::Vector3d(-22, 0, n);
    return temp;
}

Eigen::Vector3d AnkleSolver::get_F(double theta) {
    Eigen::Vector3d temp(-r_F, -d1, 0);
    temp = apply_rotation_y(temp, theta);
    temp = temp + Eigen::Vector3d(-22, 0, m);
    return temp;
}

double AnkleSolver::get_DF(double pitch, double roll, double theta_f) {
    return get_distance(get_D_after_rotation(pitch, roll), get_F(theta_f));
}

double AnkleSolver::get_CE(double pitch, double roll, double theta_e) {
    return get_distance(get_C_after_rotation(pitch, roll), get_E(theta_e));
}

double AnkleSolver::fsolve(std::function<double(double)> func, double initial_guess) {
    // Newton-Raphson method for root finding
    const double tolerance = 1e-9;
    const int max_iterations = 100;
    const double h = 1e-6; // step size for numerical derivative
    
    double x = initial_guess;
    
    for (int i = 0; i < max_iterations; ++i) {
        double fx = func(x);
        
        if (std::abs(fx) < tolerance) {
            return x;
        }
        
        // Numerical derivative: f'(x) ≈ (f(x+h) - f(x)) / h
        double fxh = func(x + h);
        double dfx = (fxh - fx) / h;
        
        if (std::abs(dfx) < 1e-12) {
            // Derivative too small, might be at a local minimum/maximum
            break;
        }
        
        // Newton-Raphson update: x_new = x - f(x) / f'(x)
        double x_new = x - fx / dfx;
        
        // Check for convergence
        if (std::abs(x_new - x) < tolerance) {
            return x_new;
        }
        
        x = x_new;
    }
    
    return x;
}

std::pair<double, double> AnkleSolver::solve(double pitch, double roll) {
    // Solve for theta_f
    auto constraint_equation_f = [this, pitch, roll](double theta_f) {
        return get_distance(get_D_after_rotation(pitch, roll), get_F(theta_f)) - h1;
    };
    
    double theta_f = fsolve(constraint_equation_f, 0.0);
    std::cout << "theta_f: " << theta_f << std::endl;
    std::cout << "constraint_equation(theta_f): " << constraint_equation_f(theta_f) << std::endl;
    
    // Solve for theta_e
    auto constraint_equation_e = [this, pitch, roll](double theta_e) {
        return get_distance(get_E(theta_e), get_C_after_rotation(pitch, roll)) - h2;
    };
    
    double theta_e = fsolve(constraint_equation_e, 0.0);
    std::cout << "theta_e: " << theta_e << std::endl;
    std::cout << "constraint_equation(theta_e): " << constraint_equation_e(theta_e) << std::endl;
    
    return std::make_pair(theta_f, theta_e);
}

