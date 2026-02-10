#include "ankle_solver.h"
#include <iostream>
#include <iomanip>

AnkleSolver::AnkleSolver() {
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
    Eigen::Vector3d temp_d(-69, -35, 0);
    return get_coord_after_pitch_roll(temp_d, pitch, roll);
}

Eigen::Vector3d AnkleSolver::get_C_after_rotation(double pitch, double roll) {
    Eigen::Vector3d temp_c(-69, 35, 0);
    return get_coord_after_pitch_roll(temp_c, pitch, roll);
}

Eigen::Vector3d AnkleSolver::get_E(double theta) {
    Eigen::Vector3d temp(-65, 0, 0);
    temp = apply_rotation_y(temp, theta);
    temp = temp + Eigen::Vector3d(-22, 34.5, 313.837);
    return temp;
}

Eigen::Vector3d AnkleSolver::get_F(double theta) {
    Eigen::Vector3d temp(-65, 0, 0);
    temp = apply_rotation_y(temp, theta);
    temp = temp + Eigen::Vector3d(-12, -34.5, 232.273);
    return temp;
}

double AnkleSolver::get_DF(double pitch, double roll, double theta_f) {
    return get_distance(get_D_after_rotation(pitch, roll), get_F(theta_f));
}

double AnkleSolver::get_CE(double pitch, double roll, double theta_e) {
    return get_distance(get_C_after_rotation(pitch, roll), get_E(theta_e));
}

double AnkleSolver::fsolve(std::function<double(double)> func, double initial_guess) {
    // Newton-Raphson method for root finding (1D)
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

std::pair<double, double> AnkleSolver::fsolve_2d(
    std::function<Eigen::Vector2d(const Eigen::Vector2d&)> func,
    const Eigen::Vector2d& initial_guess) {
    // Newton-Raphson method for 2D root finding
    const double tolerance = 1e-9;
    const int max_iterations = 100;
    const double h = 1e-6; // step size for numerical Jacobian
    
    Eigen::Vector2d x = initial_guess;
    
    for (int i = 0; i < max_iterations; ++i) {
        Eigen::Vector2d fx = func(x);
        
        // Check convergence
        if (fx.norm() < tolerance) {
            return std::make_pair(x(0), x(1));
        }
        
        // Compute numerical Jacobian: J[i][j] = ∂f_i/∂x_j
        Eigen::Matrix2d jacobian;
        
        // Partial derivatives with respect to x[0]
        Eigen::Vector2d x_plus_h0 = x;
        x_plus_h0(0) += h;
        Eigen::Vector2d fx_plus_h0 = func(x_plus_h0);
        jacobian.col(0) = (fx_plus_h0 - fx) / h;
        
        // Partial derivatives with respect to x[1]
        Eigen::Vector2d x_plus_h1 = x;
        x_plus_h1(1) += h;
        Eigen::Vector2d fx_plus_h1 = func(x_plus_h1);
        jacobian.col(1) = (fx_plus_h1 - fx) / h;
        
        // Check if Jacobian is singular
        if (std::abs(jacobian.determinant()) < 1e-12) {
            break;
        }
        
        // Newton-Raphson update: x_new = x - J^(-1) * f(x)
        Eigen::Vector2d delta = jacobian.colPivHouseholderQr().solve(fx);
        Eigen::Vector2d x_new = x - delta;
        
        // Check for convergence
        if (delta.norm() < tolerance) {
            return std::make_pair(x_new(0), x_new(1));
        }
        
        x = x_new;
    }
    
    return std::make_pair(x(0), x(1));
}

std::pair<double, double> AnkleSolver::solve(double pitch, double roll) {
    // Solve for theta_f: constraint is distance(D_after_rotation, F(theta_f)) - 232.273
    auto constraint_equation_f = [this, pitch, roll](double theta_f) {
        return get_distance(get_D_after_rotation(pitch, roll), get_F(theta_f)) - 232.273;
    };
    
    double theta_f = fsolve(constraint_equation_f, 0.0);
    // std::cout << "theta_f: " << theta_f << std::endl;
    // std::cout << "constraint_equation(theta_f): " << constraint_equation_f(theta_f) << std::endl;
    
    // Solve for theta_e: constraint is distance(E(theta_e), C_after_rotation) - 313.837
    auto constraint_equation_e = [this, pitch, roll](double theta_e) {
        return get_distance(get_E(theta_e), get_C_after_rotation(pitch, roll)) - 313.837;
    };
    
    double theta_e = fsolve(constraint_equation_e, 0.0);
    // std::cout << "theta_e: " << theta_e << std::endl;
    // std::cout << "constraint_equation(theta_e): " << constraint_equation_e(theta_e) << std::endl;
    
    return std::make_pair(theta_f, theta_e);
}

std::pair<double, double> AnkleSolver::solve_inverse(double theta_e, double theta_f) {
    // Inverse solve: given theta_e and theta_f, compute pitch and roll
    // We need to solve two constraint equations:
    // 1. distance(D_after_rotation(pitch, roll), F(theta_f)) = 232.273
    // 2. distance(E(theta_e), C_after_rotation(pitch, roll)) = 313.837
    
    auto constraint_equations = [this, theta_e, theta_f](const Eigen::Vector2d& x) {
        double pitch = x(0);
        double roll = x(1);
        
        // Constraint 1: distance(D_after_rotation, F(theta_f)) = 232.273
        double residual_f = get_distance(
            get_D_after_rotation(pitch, roll),
            get_F(theta_f)
        ) - 232.273;
        
        // Constraint 2: distance(E(theta_e), C_after_rotation) = 313.837
        double residual_e = get_distance(
            get_E(theta_e),
            get_C_after_rotation(pitch, roll)
        ) - 313.837;
        
        Eigen::Vector2d result;
        result << residual_f, residual_e;
        return result;
    };
    
    // Initial guess for pitch and roll
    Eigen::Vector2d initial_guess(0.0, 0.0);
    
    std::pair<double, double> solution = fsolve_2d(constraint_equations, initial_guess);
    double pitch = solution.first;
    double roll = solution.second;
    
    // std::cout << "pitch: " << pitch << ", roll: " << roll << std::endl;
    // Eigen::Vector2d residuals = constraint_equations(Eigen::Vector2d(pitch, roll));
    // std::cout << "constraint_equations([pitch, roll]): [" << residuals(0) << ", " << residuals(1) << "]" << std::endl;
    
    return std::make_pair(pitch, roll);
}

