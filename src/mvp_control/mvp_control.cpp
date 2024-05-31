/*
    This file is part of MVP-Control program.

    MVP-Control is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MVP-Control is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MVP-Control.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#include "mvp_control.h"
#include "ros/ros.h"
#include "mvp_control/dictionary.h"
#include "exception.hpp"
#include "functional"

using namespace ctrl;

MvpControl::MvpControl() {

    /**
     * Initialize the error state
     */
    m_error_state = Eigen::VectorXd(CONTROLLABLE_DOF_LENGTH);

    /**
     * Create the multiple input multiple output PID controller
     */
    m_pid.reset(new MimoPID());

    /**
     * MIMO PID object does not implement the error function. It asks programmer
     * to assign a error function. When it calculates the gains it uses the
     * function that is binded to it's error function.
     */
    m_pid->set_error_function(
        std::bind(
            &MvpControl::f_error_function,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

}

void MvpControl::set_control_allocation_matrix(
        const decltype(m_control_allocation_matrix)& matrix) {
    m_control_allocation_matrix = matrix;
}

auto MvpControl::get_control_allocation_matrix() ->
        decltype(m_control_allocation_matrix) {
    return m_control_allocation_matrix;
}

void MvpControl::set_thruster_articulation_vector(
        const decltype(m_thruster_vector)& vector) {
    m_thruster_vector = vector;
}

auto MvpControl::get_thruster_articulation_vector() ->
        decltype(m_thruster_vector){
    return m_thruster_vector;
}

void MvpControl::set_controller_frequency(
        const decltype(m_controller_frequency)& frequency) {
    m_controller_frequency = frequency;
}

auto MvpControl::get_pid() -> decltype(m_pid) {
    return m_pid;
}

void MvpControl::set_pid(const MimoPID::Ptr &pid) {
    m_pid = pid;
}

auto MvpControl::get_system_state() -> decltype(m_system_state) {
    return m_system_state;
}

void MvpControl::set_system_state(
        const decltype(m_system_state) &system_state) {
    m_system_state = system_state;
}

auto MvpControl::get_desired_state() -> decltype(m_desired_state) {
    return m_desired_state;
}

void MvpControl::set_desired_state(
        const decltype(m_desired_state) &desired_state) {
    m_desired_state = desired_state;
}

void MvpControl::set_lower_limit(const decltype(m_lower_limit) &lower_limit) {
    m_lower_limit = lower_limit;
    //std::cout << "Lower limit set to: " << m_lower_limit.transpose() << std::endl;
}

void MvpControl::set_upper_limit(const decltype(m_upper_limit) &upper_limit) {
    m_upper_limit = upper_limit;
    //std::cout << "Upper limit set to: " << m_upper_limit.transpose() << std::endl;
}

void MvpControl::set_current_angle(const int* m_thruster_index, double angle) {
    // if (*m_thruster_index < 0 || *m_thruster_index >= m_current_angles.size()) {
    //     throw std::out_of_range("Thruster index is out of range");
    // }
    m_current_angles[*m_thruster_index] = angle;
}

double MvpControl::get_current_angle(const int* m_thruster_index) const {
    // if (*m_thruster_index < 0 || *m_thruster_index >= m_current_angles.size()) {
    //     throw std::out_of_range("Thruster index is out of range");
    // }
    return m_current_angles[*m_thruster_index];
}

bool MvpControl::calculate_needed_forces(Eigen::VectorXd *f, double dt) {

    /**
     * This function basically computes one iteration of the controller. It
     * computes the PID gains. Optimizes the thrust.
     */

    /**
     * vector 'u' represents the input values of the system. That is all degrees
     * of freedoms.
     */
    Eigen::VectorXd u;
    if(!f_calculate_pid(&u, dt)){
        return false;
    }

    // Convert Eigen::VectorXd to string using std::stringstream
    // std::stringstream ss;
    // ss << u.transpose();  // Use transpose to print the vector horizontally

    // Log the content of 'u' using ROS_INFO
    //ROS_INFO_STREAM("Current input values (u): " << ss.str());
    // std::cout << "Current input values (u): \n"<<u<<std::endl;
        /**
     * Below code computes the forces that later will be requested from the
     * thrusters. Values in the force vector are not thruster set points yet.
     */
    if (f_optimize_thrust(f, u))
    {
        return true;
    }
    else
    {
        ROS_WARN("Optimization of thrust failed!");
    }
    return false;
}

bool MvpControl::f_calculate_pid(Eigen::VectorXd *u, double dt)
{
    return m_pid->calculate(u, m_desired_state, m_system_state, dt);
}

double normalize_angle(double angle) {
// Normalize angle to be within the range [0, 2 * M_PI)
    double two_pi = 2 * M_PI;
    return angle - two_pi * std::floor(angle / two_pi);
}

bool MvpControl::f_optimize_thrust(Eigen::VectorXd *t, Eigen::VectorXd u) {

    // Static boolean flag to ensure initialization only happens once
    static bool is_initialized = false;
    if (!is_initialized) {
        m_current_angles.resize(m_thruster_vector.size(), 0.0);
        is_initialized = true;
    }

    // Check sizes of the inputs
    // ROS_INFO_STREAM("Size of m_controlled_freedoms: " << m_controlled_freedoms.size());
    // ROS_INFO_STREAM("Size of m_control_allocation_matrix: " << m_control_allocation_matrix.rows() << "x" << m_control_allocation_matrix.cols());
    // ROS_INFO_STREAM("Size of u: " << u.size());

    // Control allocation matrix
    Eigen::MatrixXd T(m_controlled_freedoms.size(), m_control_allocation_matrix.cols());

    // Control matrix
    Eigen::VectorXd U(m_controlled_freedoms.size());

    // Scoped lock to ensure thread safety
    {
        std::scoped_lock lock(m_allocation_matrix_lock, m_controlled_freedoms_lock);
        for (int i = 0; i < m_controlled_freedoms.size(); i++) {
            int idx = m_controlled_freedoms.at(i);
            if (idx < 0 || idx >= m_control_allocation_matrix.rows() || idx >= u.size()) {
                ROS_ERROR_STREAM("Index out of bounds when accessing controlled freedoms: " << idx);
                return false;
            }
            //ROS_INFO_STREAM("Accessing row: " << idx);
            T.row(i) = m_control_allocation_matrix.row(idx);
            U(i) = u(idx);
        }
    }

    // Prepare data for the quadratic solver
    Eigen::MatrixXd Q = 2 * T.transpose() * T;
    Eigen::VectorXd c = -2 * T.transpose() * U;

    // Print the m_thruster_vector
    ROS_INFO_STREAM("m_thruster_vector: " << m_thruster_vector.transpose());

    // Print the contents of m_current_angles
    std::stringstream ss;
    ss << "Current Angles vector: ";
    for (const auto& angle : m_current_angles) {
        ss << angle << " ";
    }
    ROS_INFO_STREAM(ss.str());

    // Initialize constraint matrix and bounds
    const double kInfinity = std::numeric_limits<double>::infinity();

    const double omega = 5.24;
    const double gamma_lower = -2.1; //-120deg (6.28-3.10 radians)
    const double gamma_upper = 2.1; //120deg
    const double deltaT = 1 / m_controller_frequency;
    int pair_count = 0;
    int single_count = 0;

    for (size_t i = 0; i + 1 < m_thruster_vector.size(); ++i) {
        if (m_thruster_vector(i) == 1 && m_thruster_vector(i + 1) == 2) {
            pair_count++;  // Increment for every [1, 2] pair found
            i++;  // Skip the next element because it is part of a pair
        } else {
            single_count++;  // Not part of a [1, 2] pair
        }
    }

    // Check the last element if it's not part of the last checked pair
    if (m_thruster_vector.size() > 0 && (
        m_thruster_vector(m_thruster_vector.size() - 1) != 2 || (m_thruster_vector.size() > 1
        && m_thruster_vector(m_thruster_vector.size() - 2) != 1))) {
        single_count++;
    }

    int kNumConstraints = 5 * pair_count + single_count;
    int kNumVariables = m_control_allocation_matrix.cols();

    //ROS_INFO_STREAM("Number of constraints: " << kNumConstraints);
    //ROS_INFO_STREAM("Number of variables: " << kNumVariables);

    // Initialize the OSQP instance
    osqp::OsqpInstance qp_instance;
    qp_instance.objective_matrix = Q.sparseView();
    qp_instance.objective_vector = c;
    qp_instance.lower_bounds.resize(kNumConstraints);
    qp_instance.upper_bounds.resize(kNumConstraints);

    Eigen::SparseMatrix<double> A_sparse(kNumConstraints, kNumVariables);
    std::vector<Eigen::Triplet<double>> A_triplets;
    A_triplets.reserve(m_thruster_vector.size() * 9); // Estimate the required size

    const double omega_deltaT = omega * deltaT;
    double beta = 0.0;



    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        int thruster_setting = static_cast<int>(m_thruster_vector[i]);
        //std::string joint_name = m_tf_prefix_thruster; // + m_thrusters.at(i)->get_servo_joints().at(0);

        beta = m_current_angles[i];
        int base_idx = 0;

        //ROS_INFO("m_tf_prefix_thruster: %s", m_tf_prefix_thruster.c_str());
        //ROS_INFO("Haloooooooooooooooooooo");
        //ROS_INFO("Current angle beta: %f", beta);
        double normalized_alpha_lower_beta = normalize_angle(omega_deltaT + beta);
        double normalized_alpha_upper_beta = normalize_angle(-omega_deltaT + beta);

        switch (thruster_setting) {
            case 0:
                //ROS_ERROR_STREAM("Index is this honey :" << i);
                A_triplets.emplace_back(i + 6, i, 1.0);
                qp_instance.lower_bounds[i + 6] = m_lower_limit[i];
                qp_instance.upper_bounds[i + 6] = m_upper_limit[i];
                break;
            case 1:
                //ROS_ERROR_STREAM("Index is this honey :" << i);
                base_idx = (i * 5) / 2;
                // Check for potential overwrites
                if (base_idx >= qp_instance.lower_bounds.size() || base_idx + 4 >= qp_instance.upper_bounds.size()) {
                    ROS_ERROR_STREAM("Index out of bounds when setting bounds for 1: " << base_idx << " or " << base_idx + 4);
                    return false;
                }

                A_triplets.emplace_back(base_idx, i, 1.0);
                A_triplets.emplace_back(base_idx + 1, i, -omega_deltaT);
                A_triplets.emplace_back(base_idx + 2, i, -omega_deltaT);
                A_triplets.emplace_back(base_idx + 1, i + 1, 1.0);
                A_triplets.emplace_back(base_idx + 2, i + 1, -1.0);
                A_triplets.emplace_back(base_idx + 3, i, -std::min(omega_deltaT, gamma_upper - beta));
                A_triplets.emplace_back(base_idx + 4, i, std::max(-omega_deltaT, gamma_lower + beta));
                //A_triplets.emplace_back(base_idx + 3, i, tan(-std::min(gamma_upper, normalized_alpha_upper_beta)));
                //A_triplets.emplace_back(base_idx + 4, i, tan(std::max(gamma_lower, normalized_alpha_lower_beta)));
                A_triplets.emplace_back(base_idx + 3, i + 1, 1.0);
                A_triplets.emplace_back(base_idx + 4, i + 1, -1.0);

                // Log the values
               //ROS_INFO_STREAM("base_idx: " << base_idx);
                ROS_INFO_STREAM("i: " << i);
                ROS_INFO_STREAM("omega_deltaT: " << omega_deltaT);
                ROS_INFO_STREAM("gamma_upper: " << gamma_upper);
                ROS_INFO_STREAM("gamma_lower: " << gamma_lower);
                ROS_INFO_STREAM("beta: " << beta);
                ROS_INFO_STREAM("-std::min(omega_deltaT, gamma_upper - beta)): " << -std::min(omega_deltaT, gamma_upper - beta));
                ROS_INFO_STREAM("std::max(-omega_deltaT, gamma_lower + beta)): " << std::max(-omega_deltaT, gamma_lower + beta));

                // Set bounds for the constraints associated with thruster_setting 1
                if (base_idx >= qp_instance.lower_bounds.size() || base_idx + 4 >= qp_instance.upper_bounds.size()) {
                    ROS_ERROR_STREAM("Index out of bounds when setting bounds for 1: " << base_idx << " or " << base_idx + 4);
                    return false;
                }
                qp_instance.lower_bounds[base_idx] = 0;
                qp_instance.upper_bounds[base_idx] = m_upper_limit[(base_idx / 3) + 1] * std::cos(omega_deltaT);

                qp_instance.lower_bounds[base_idx + 1] = -kInfinity;
                qp_instance.upper_bounds[base_idx + 1] = 0;
                qp_instance.lower_bounds[base_idx + 2] = -kInfinity;
                qp_instance.upper_bounds[base_idx + 2] = 0;
                qp_instance.lower_bounds[base_idx + 3] = -kInfinity;
                qp_instance.upper_bounds[base_idx + 3] = 0;
                qp_instance.lower_bounds[base_idx + 4] = -kInfinity;
                qp_instance.upper_bounds[base_idx + 4] = 0;
                break;
            case 2:
                // No action required
                break;
            default:
                ROS_ERROR_STREAM("Unexpected thruster setting: " << thruster_setting);
                return false;
        }
    }

    A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
    qp_instance.constraint_matrix = A_sparse;

    // Debug bounds
    for (int i = 0; i < qp_instance.lower_bounds.size(); ++i) {
       // ROS_INFO_STREAM("Lower bound [" << i << "]: " << qp_instance.lower_bounds[i]);
    }
    for (int i = 0; i < qp_instance.upper_bounds.size(); ++i) {
        //ROS_INFO_STREAM("Upper bound [" << i << "]: " << qp_instance.upper_bounds[i]);
    }

    // Solve the quadratic program
    osqp::OsqpSolver solver;
    osqp::OsqpSettings settings;
    settings.verbose = false;
    auto status = solver.Init(qp_instance, settings);

    if (!status.ok()) {
        ROS_ERROR("OSQP solver initialization failed.");
        return false;
    }

    osqp::OsqpExitCode exitCode = solver.Solve();

    switch (exitCode) {
        case osqp::OsqpExitCode::kOptimal: {
            *t = solver.primal_solution();
            return true;
        }
        case osqp::OsqpExitCode::kPrimalInfeasible:
            ROS_ERROR("The problem is primal infeasible.");
            break;
        case osqp::OsqpExitCode::kDualInfeasible:
            ROS_ERROR("The problem is dual infeasible.");
            break;
        case osqp::OsqpExitCode::kOptimalInaccurate:
            ROS_ERROR("The optimal solution is inaccurate.");
            break;
        case osqp::OsqpExitCode::kPrimalInfeasibleInaccurate:
            ROS_ERROR("The problem is primal infeasible and the solution is inaccurate.");
            break;
        case osqp::OsqpExitCode::kDualInfeasibleInaccurate:
            ROS_ERROR("The problem is dual infeasible and the solution is inaccurate.");
            break;
        case osqp::OsqpExitCode::kMaxIterations:
            ROS_ERROR("The maximum number of iterations has been reached.");
            break;
        case osqp::OsqpExitCode::kInterrupted:
            ROS_ERROR("The optimization was interrupted.");
            break;
        case osqp::OsqpExitCode::kTimeLimitReached:
            ROS_ERROR("The time limit was reached before a solution was found.");
            break;
        case osqp::OsqpExitCode::kNonConvex:
            ROS_ERROR("The problem is non-convex.");
            break;
        case osqp::OsqpExitCode::kUnknown:
        default:
            ROS_ERROR("An unknown error occurred.");
            break;
    }

    return false;
}

// bool MvpControl::f_optimize_thrust(Eigen::VectorXd *t, Eigen::VectorXd u) {

//     ROS_INFO_STREAM("Size of m_controlled_freedoms: " << m_controlled_freedoms.size());
//     ROS_INFO_STREAM("Size of m_control_allocation_matrix: " << m_control_allocation_matrix.rows() << "x" << m_control_allocation_matrix.cols());
//     ROS_INFO_STREAM("Size of u: " << u.size());

//     // Control allocation matrix
//     Eigen::MatrixXd T(
//         m_controlled_freedoms.size(),
//         m_control_allocation_matrix.cols()
//     );

//     // Control matrix
//     Eigen::VectorXd U(m_controlled_freedoms.size());

//     //std::vector<std::string> m_controlled_freedoms_names = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
//     // // Scoped lock to ensure thread safety
//     // {
//     //     std::scoped_lock lock(m_allocation_matrix_lock, m_controlled_freedoms_lock);
//     //     for (int i = 0; i < m_controlled_freedoms.size(); i++) {
//     //         T.row(i) = m_control_allocation_matrix.row(m_controlled_freedoms.at(i));
//     //         U(i) = u(m_controlled_freedoms.at(i));

//     //     }
//     //         // std::stringstream ss;
//     //         //     ss << "U: [" << U.transpose() << "]";
//     //         //     ROS_INFO_STREAM(ss.str());
//     // }

//     // Scoped lock to ensure thread safety
//     {
//         std::scoped_lock lock(m_allocation_matrix_lock, m_controlled_freedoms_lock);
//         for (int i = 0; i < m_controlled_freedoms.size(); i++) {
//             int idx = m_controlled_freedoms.at(i);
//             if (idx < 0 || idx >= m_control_allocation_matrix.rows() || idx >= u.size()) {
//                 ROS_ERROR_STREAM("Index out of bounds: " << idx);
//                 return false;
//             }
//             ROS_INFO_STREAM("Accessing row: " << idx);
//             T.row(i) = m_control_allocation_matrix.row(idx);
//             U(i) = u(idx);
//         }
//     }
    
//     // Prepare data for the quadratic solver
//     Eigen::MatrixXd Q = 2 * T.transpose() * T;
//     Eigen::VectorXd c = -2 * T.transpose() * U;

//     // Initialize constraint matrix and bounds
//     const double kInfinity = std::numeric_limits<double>::infinity();

//     const double omega = 5.24;
//     const double gamma_lower = 0.01;
//     const double gamma_upper = 0.01;
//     const double deltaT = 1 / m_controller_frequency;
//     int pair_count = 0;
//     int single_count = 0;

//     for (size_t i = 0; i + 1 < m_thruster_vector.size(); ++i) {
//         if (m_thruster_vector(i) == 1 && m_thruster_vector(i + 1) == 2) {
//             pair_count++;  // Increment for every [1, 2] pair found
//             i++;  // Skip the next element because it is part of a pair
//         } else {
//             single_count++;  // Not part of a [1, 2] pair
//         }
//     }

//     // Check the last element if it's not part of the last checked pair
//     if (m_thruster_vector.size() > 0 && (
//         m_thruster_vector(m_thruster_vector.size() - 1) != 2 || (m_thruster_vector.size() > 1
//         && m_thruster_vector(m_thruster_vector.size() - 2) != 1))) {
//         single_count++;
//     }

//     // @TODO dynamic beta
//     // int kNumConstraints = 5 * pair_count + single_count;
//     // int kNumVariables = m_control_allocation_matrix.cols();

//     // // Initialize the OSQP instance
//     // osqp::OsqpInstance qp_instance;
//     // qp_instance.objective_matrix = Q.sparseView();
//     // qp_instance.objective_vector = c;
//     // qp_instance.lower_bounds.resize(kNumConstraints);
//     // qp_instance.upper_bounds.resize(kNumConstraints);

//     // Eigen::SparseMatrix<double> A_sparse(kNumConstraints, kNumVariables);
//     // std::vector<Eigen::Triplet<double>> A_triplets;

//     // //Assuming articulated joints are entered first in config file
//     // // followed by non articulated
//     // for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
//     //     int thruster_setting = static_cast<int>(m_thruster_vector[i]);

//     //     switch (thruster_setting) {
//     //         case 0:
//     //             A_triplets.emplace_back((i * 3)/2, i, 1.0);
//     //             break;
//     //         case 1:
//     //             A_triplets.emplace_back((i * 5)/2, i, 1.0);
//     //             A_triplets.emplace_back((i * 5)/2 + 1, i, -omega * deltaT);
//     //             A_triplets.emplace_back((i * 5)/2 + 2, i, -omega * deltaT);
//     //             A_triplets.emplace_back((i * 5)/2 + 1, i + 1, 1.0);
//     //             A_triplets.emplace_back((i * 5)/2+ 2, i + 1, -1.0);
//     //             A_triplets.emplace_back((i * 5)/2 + 3, i, -min(alpha,gamma+beta));
//     //             A_triplets.emplace_back((i * 5)/2 + 4, i, max(-alpha,-gamma+beta));
//     //             A_triplets.emplace_back((i * 5)/2 + 3, i + 1, 1.0);
//     //             A_triplets.emplace_back((i * 5)/2+ 4, i + 1, -1.0);
//     //             break;
//     //         case 2:
//     //             break;
//     //         default:
//     //             // Handling unexpected settings
//     //             break;
//     //     }
//     // }

//     // A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
//     // qp_instance.constraint_matrix = A_sparse;

//     // // making this as a helper to build boundaries
//     // std::vector<int> boundary_slack_vector;

//     // for (int i = 0; i < m_thruster_vector.size(); ++i) {
//     //     int value = static_cast<int>(m_thruster_vector[i]);
//     //     switch (value) {
//     //         case 1:
//     //             boundary_slack_vector.push_back(1);
//     //             boundary_slack_vector.push_back(1);
//     //             boundary_slack_vector.push_back(1);
//     //             boundary_slack_vector.push_back(1);
//     //             boundary_slack_vector.push_back(1);
//     //             break;
//     //         case 0:
//     //             boundary_slack_vector.push_back(0);
//     //             break;
//     //         case 2:
//     //             // do nothing
//     //             break;
//     //         default:
//     //             // Handle unexpected values
//     //             break;
//     //     }
//     // }

//     // for (size_t i = 0; i < boundary_slack_vector.size(); ++i) {
//     //     switch (static_cast<int>(boundary_slack_vector[i])) {
//     //         case 1:
//     //             // F1 and F2 x
//     //             qp_instance.lower_bounds[i] = 0;
//     //             qp_instance.upper_bounds[i] =  m_upper_limit((i/3)+1) * cos(omega * deltaT);

//     //             for (int j = i + 1; j < i + 5; ++j) {
//     //                 qp_instance.lower_bounds[j] = -kInfinity;
//     //                 qp_instance.upper_bounds[j] = 0;
//     //             }
//     //             i+=3;
//     //             break;
//     //         case 0:
//     //             if ((i + 1) % 5 == 0) {
//     //                 qp_instance.lower_bounds[i] = m_lower_limit[i-4];
//     //                 qp_instance.upper_bounds[i] = m_upper_limit[i-4];
//     //             } else {
//     //                 qp_instance.lower_bounds[i] = m_lower_limit[i-5];
//     //                 qp_instance.upper_bounds[i] = m_upper_limit[i-5];
//     //             }
//     //             break;
//     //     }
//     // }

//     int kNumConstraints = 5 * pair_count + single_count;
//     int kNumVariables = m_control_allocation_matrix.cols();

//     // Initialize the OSQP instance
//     osqp::OsqpInstance qp_instance;
//     qp_instance.objective_matrix = Q.sparseView();
//     qp_instance.objective_vector = c;
//     qp_instance.lower_bounds.resize(kNumConstraints);
//     qp_instance.upper_bounds.resize(kNumConstraints);

//     Eigen::SparseMatrix<double> A_sparse(kNumConstraints, kNumVariables);
//     std::vector<Eigen::Triplet<double>> A_triplets;
//     A_triplets.reserve(m_thruster_vector.size() * 9); // Estimate the required size

//     const double omega_deltaT = omega * deltaT;

//     for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
//         int thruster_setting = static_cast<int>(m_thruster_vector[i]);
//         std::string joint_name = m_tf_prefix_thruster ;// + m_thrusters.at(i)->get_servo_joints().at(0);
//         //double beta = get_current_angle(joint_name); // Retrieve the current angle
//         double beta = 0.0;
//         int base_idx = 0;

//         //ROS_INFO("m_tf_prefix_thruster: %s", m_tf_prefix_thruster.c_str());

//         switch (thruster_setting) {
//             case 0:

//                 if ((i * 3) / 2 >= kNumConstraints || i >= kNumVariables) {
//                         ROS_ERROR_STREAM("Index out of bounds: " << (i * 3) / 2 << " or " << i);
//                         return false;
//                     }

//                 A_triplets.emplace_back((i * 3) / 2, i, 1.0);
//                 qp_instance.lower_bounds[(i * 3) / 2] = m_lower_limit[(i * 3) / 2];
//                 qp_instance.upper_bounds[(i * 3) / 2] = m_upper_limit[(i * 3) / 2];
//                 break;
//             case 1:
//                 base_idx = (i * 5) / 2;
//                 if (base_idx + 4 >= kNumConstraints || i + 1 >= kNumVariables) {
//                     ROS_ERROR_STREAM("Index out of bounds: " << base_idx + 4 << " or " << i + 1);
//                     return false;
//                 }
//                 base_idx = (i * 5) / 2;
//                 A_triplets.emplace_back(base_idx, i, 1.0);
//                 A_triplets.emplace_back(base_idx + 1, i, -omega_deltaT);
//                 A_triplets.emplace_back(base_idx + 2, i, -omega_deltaT);
//                 A_triplets.emplace_back(base_idx + 1, i + 1, 1.0);
//                 A_triplets.emplace_back(base_idx + 2, i + 1, -1.0);
//                 A_triplets.emplace_back(base_idx + 3, i, -std::min(omega_deltaT, gamma_upper + beta));
//                 A_triplets.emplace_back(base_idx + 4, i, std::max(-omega_deltaT, gamma_lower + beta));
//                 A_triplets.emplace_back(base_idx + 3, i + 1, 1.0);
//                 A_triplets.emplace_back(base_idx + 4, i + 1, -1.0);

//                 // Set bounds for the constraints associated with thruster_setting 1
//                 qp_instance.lower_bounds[base_idx] = 0;
//                 qp_instance.upper_bounds[base_idx] = m_upper_limit[(base_idx / 3) + 1] * std::cos(omega_deltaT);

//                 qp_instance.lower_bounds[base_idx + 1] = -kInfinity;
//                 qp_instance.upper_bounds[base_idx + 1] = 0;
//                 qp_instance.lower_bounds[base_idx + 2] = -kInfinity;
//                 qp_instance.upper_bounds[base_idx + 2] = 0;
//                 qp_instance.lower_bounds[base_idx + 3] = -kInfinity;
//                 qp_instance.upper_bounds[base_idx + 3] = 0;
//                 qp_instance.lower_bounds[base_idx + 4] = -kInfinity;
//                 qp_instance.upper_bounds[base_idx + 4] = 0;
//                 break;
//             case 2:
//                 // No action required
//                 break;
//             default:
//                 // Handle unexpected settings
//                 break;
//         }
//     }

//     A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
//     qp_instance.constraint_matrix = A_sparse;

//     // Solve the quadratic program
//     osqp::OsqpSolver solver;
//     osqp::OsqpSettings settings;
//     settings.verbose = false;
//     auto status = solver.Init(qp_instance, settings);

//     if (!status.ok()) {
//         return false;
//     }

//     osqp::OsqpExitCode exitCode = solver.Solve();

//     switch (exitCode) {
//         case osqp::OsqpExitCode::kOptimal: {
//             *t = solver.primal_solution();

//             // Output the optimal solution
//             // std::cout << "Optimal solution vector: [" << t->transpose() << "]" << std::endl;
//             return true;
//         }
//         case osqp::OsqpExitCode::kPrimalInfeasible:
//             std::cerr << "The problem is primal infeasible." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kDualInfeasible:
//             std::cerr << "The problem is dual infeasible." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kOptimalInaccurate:
//             std::cerr << "The optimal solution is inaccurate." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kPrimalInfeasibleInaccurate:
//             std::cerr << "The problem is primal infeasible and the solution is inaccurate." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kDualInfeasibleInaccurate:
//             std::cerr << "The problem is dual infeasible and the solution is inaccurate." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kMaxIterations:
//             std::cerr << "The maximum number of iterations has been reached." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kInterrupted:
//             std::cerr << "The optimization was interrupted." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kTimeLimitReached:
//             std::cerr << "The time limit was reached before a solution was found." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kNonConvex:
//             std::cerr << "The problem is non-convex." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kUnknown:
//         default:
//             std::cerr << "An unknown error occurred." << std::endl;
//             break;
//     }

//     return false;
// }

// bool MvpControl::f_optimize_thrust_2(Eigen::VectorXd *t, Eigen::VectorXd u) {

//     // Control allocation matrix
//     Eigen::MatrixXd T(
//         m_controlled_freedoms.size(),
//         m_control_allocation_matrix.cols()
//     );
    
//     // Control matrix
//     Eigen::VectorXd U(m_controlled_freedoms.size());

//     //std::vector<std::string> m_controlled_freedoms_names = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
//     // Scoped lock to ensure thread safety
//     {
//         std::scoped_lock lock(m_allocation_matrix_lock, m_controlled_freedoms_lock);
//         for (int i = 0; i < m_controlled_freedoms.size(); i++) {
//             T.row(i) = m_control_allocation_matrix.row(m_controlled_freedoms.at(i));
//             U(i) = u(m_controlled_freedoms.at(i));

//         }
//             // std::stringstream ss;
//             //     ss << "U: [" << U.transpose() << "]";
//             //     ROS_INFO_STREAM(ss.str());
//     }

//     // Prepare data for the quadratic solver
//     Eigen::MatrixXd Q = 2 * T.transpose() * T;
//     Eigen::VectorXd c = -2 * T.transpose() * U;

//     // Initialize constraint matrix and bounds
//     const double kInfinity = std::numeric_limits<double>::infinity();

//     const double omega = 5.24;
//     const double deltaT = 1 / m_controller_frequency;
//     int pair_count = 0;
//     int single_count = 0;

//     for (size_t i = 0; i + 1 < m_thruster_vector.size(); ++i) {
//         if (m_thruster_vector(i) == 1 && m_thruster_vector(i + 1) == 2) {
//             pair_count++;  // Increment for every [1, 2] pair found
//             i++;  // Skip the next element because it is part of a pair
//         } else {
//             single_count++;  // Not part of a [1, 2] pair
//         }
//     }

//     // Check the last element if it's not part of the last checked pair
//     if (m_thruster_vector.size() > 0 && (
//         m_thruster_vector(m_thruster_vector.size() - 1) != 2 || (m_thruster_vector.size() > 1 
//         && m_thruster_vector(m_thruster_vector.size() - 2) != 1))) {
//         single_count++;
//     }

//     int kNumConstraints = 3 * pair_count + single_count; 
//     int kNumVariables = m_control_allocation_matrix.cols();
    
//     // Initialize the OSQP instance
//     osqp::OsqpInstance qp_instance;
//     qp_instance.objective_matrix = Q.sparseView();
//     qp_instance.objective_vector = c;
//     qp_instance.lower_bounds.resize(kNumConstraints);
//     qp_instance.upper_bounds.resize(kNumConstraints);

//     Eigen::SparseMatrix<double> A_sparse(kNumConstraints, kNumVariables);
//     std::vector<Eigen::Triplet<double>> A_triplets;
 
//     //Assuming articulated joints are entered first in config file
//     // followed by non articulated
//     for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
//         int thruster_setting = static_cast<int>(m_thruster_vector[i]);
//         //std::string joint_name = m_tf_prefix ;// + m_thrusters.at(i)->get_servo_joints().at(0);
//         //double beta = get_current_angle(joint_name); // Retrieve the current angle
//         double beta = 0.0;
//         int base_idx = 0;

//         ROS_INFO("m_tf_prefix_thruster: %s", m_tf_prefix_thruster.c_str());

//         switch (thruster_setting) {
//             case 0:
//                 A_triplets.emplace_back((i * 3)/2, i, 1.0);
//                 break;
//             case 1:
//                 base_idx = (i * 5) / 2;
//                 A_triplets.emplace_back(base_idx, i, 1.0);
//                 A_triplets.emplace_back(base_idx + 1, i, -omega_deltaT);
//                 A_triplets.emplace_back(base_idx + 2, i, -omega_deltaT);
//                 A_triplets.emplace_back(base_idx + 1, i + 1, 1.0);
//                 A_triplets.emplace_back(base_idx + 2, i + 1, -1.0);
//                 //A_triplets.emplace_back(base_idx + 3, i, -std::min(omega_deltaT, gamma_upper + beta));
//                 //A_triplets.emplace_back(base_idx + 4, i, std::max(-omega_deltaT, gamma_lower + beta));
//                 A_triplets.emplace_back(base_idx + 3, i + 1, 1.0);
//                 A_triplets.emplace_back(base_idx + 4, i + 1, -1.0);

//                 // Set bounds for the constraints associated with thruster_setting 1
//                 qp_instance.lower_bounds[base_idx] = 0;
//                 qp_instance.upper_bounds[base_idx] = m_upper_limit[(base_idx / 3) + 1] * std::cos(omega_deltaT);

//                 qp_instance.lower_bounds[base_idx + 1] = -kInfinity;
//                 qp_instance.upper_bounds[base_idx + 1] = 0;
//                 qp_instance.lower_bounds[base_idx + 2] = -kInfinity;
//                 qp_instance.upper_bounds[base_idx + 2] = 0;
//                 qp_instance.lower_bounds[base_idx + 3] = -kInfinity;
//                 qp_instance.upper_bounds[base_idx + 3] = 0;
//                 qp_instance.lower_bounds[base_idx + 4] = -kInfinity;
//                 qp_instance.upper_bounds[base_idx + 4] = 0;
//                 break;
//             case 2:
//                 break;
//             default:
//                 // Handling unexpected settings
//                 break;
//         }
//     }

//     A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
//     qp_instance.constraint_matrix = A_sparse;

//     // making this as a helper to build boundaries
//     std::vector<int> boundary_slack_vector;

//     for (int i = 0; i < m_thruster_vector.size(); ++i) {
//         int value = static_cast<int>(m_thruster_vector[i]);
//         switch (value) {
//             case 1:
//                 boundary_slack_vector.push_back(1);
//                 boundary_slack_vector.push_back(1);
//                 boundary_slack_vector.push_back(1);
//                 break;
//             case 0:
//                 boundary_slack_vector.push_back(0);
//                 break;
//             case 2:
//                 // do nothing
//                 break;
//             default:
//                 // Handle unexpected values
//                 break;
//         }
//     }

//     for (size_t i = 0; i < boundary_slack_vector.size(); ++i) {
//         switch (static_cast<int>(boundary_slack_vector[i])) {
//             case 1:
//                 // F1 and F2 x
//                 qp_instance.lower_bounds[i] = 0; 
//                 qp_instance.upper_bounds[i] =  m_upper_limit((i/3)+1) * cos(omega * deltaT);

//                 for (int j = i + 1; j < i + 3; ++j) {
//                     qp_instance.lower_bounds[j] = -kInfinity;
//                     qp_instance.upper_bounds[j] = 0;
//                 }
//                 i++;
//                 i++;
//                 break;
//             case 0:
//                 if ((i + 1) % 3 == 0) {
//                     qp_instance.lower_bounds[i] = m_lower_limit[i-2]; 
//                     qp_instance.upper_bounds[i] = m_upper_limit[i-2];
//                 } else { 
//                     qp_instance.lower_bounds[i] = m_lower_limit[i-3]; 
//                     qp_instance.upper_bounds[i] = m_upper_limit[i-3];
//                 }
//                 break;
//         }
//     }

//     // Solve the quadratic program
//     osqp::OsqpSolver solver;
//     osqp::OsqpSettings settings;
//     settings.verbose = false;
//     auto status = solver.Init(qp_instance, settings);

//     if (!status.ok()) {
//         return false;
//     }

//     osqp::OsqpExitCode exitCode = solver.Solve();

//     switch (exitCode) {
//         case osqp::OsqpExitCode::kOptimal: {
//             *t = solver.primal_solution();

//             // Output the optimal solution
//             // std::cout << "Optimal solution vector: [" << t->transpose() << "]" << std::endl;
//             return true;
//         }
//         case osqp::OsqpExitCode::kPrimalInfeasible:
//             std::cerr << "The problem is primal infeasible." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kDualInfeasible:
//             std::cerr << "The problem is dual infeasible." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kOptimalInaccurate:
//             std::cerr << "The optimal solution is inaccurate." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kPrimalInfeasibleInaccurate:
//             std::cerr << "The problem is primal infeasible and the solution is inaccurate." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kDualInfeasibleInaccurate:
//             std::cerr << "The problem is dual infeasible and the solution is inaccurate." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kMaxIterations:
//             std::cerr << "The maximum number of iterations has been reached." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kInterrupted:
//             std::cerr << "The optimization was interrupted." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kTimeLimitReached:
//             std::cerr << "The time limit was reached before a solution was found." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kNonConvex:
//             std::cerr << "The problem is non-convex." << std::endl;
//             break;
//         case osqp::OsqpExitCode::kUnknown:
//         default:
//             std::cerr << "An unknown error occurred." << std::endl;
//             break;
//     }

//     return false;
// }

bool MvpControl::f_optimize_thrust_2(Eigen::VectorXd *t, Eigen::VectorXd u) {

    // Control allocation matrix
    Eigen::MatrixXd T(
        m_controlled_freedoms.size(),
        m_control_allocation_matrix.cols()
    );
    
    // Control matrix
    Eigen::VectorXd U(m_controlled_freedoms.size());

    //std::vector<std::string> m_controlled_freedoms_names = {"Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"};
    // Scoped lock to ensure thread safety
    {
        std::scoped_lock lock(m_allocation_matrix_lock, m_controlled_freedoms_lock);
        for (int i = 0; i < m_controlled_freedoms.size(); i++) {
            T.row(i) = m_control_allocation_matrix.row(m_controlled_freedoms.at(i));
            U(i) = u(m_controlled_freedoms.at(i));

        }
            // std::stringstream ss;
            //     ss << "U: [" << U.transpose() << "]";
            //     ROS_INFO_STREAM(ss.str());
    }

    // Prepare data for the quadratic solver
    Eigen::MatrixXd Q = 2 * T.transpose() * T;
    Eigen::VectorXd c = -2 * T.transpose() * U;

    // Initialize constraint matrix and bounds
    const double kInfinity = std::numeric_limits<double>::infinity();

    const double omega = 5.24;
    const double deltaT = 1 / m_controller_frequency;
    int pair_count = 0;
    int single_count = 0;

    for (size_t i = 0; i + 1 < m_thruster_vector.size(); ++i) {
        if (m_thruster_vector(i) == 1 && m_thruster_vector(i + 1) == 2) {
            pair_count++;  // Increment for every [1, 2] pair found
            i++;  // Skip the next element because it is part of a pair
        } else {
            single_count++;  // Not part of a [1, 2] pair
        }
    }

    // Check the last element if it's not part of the last checked pair
    if (m_thruster_vector.size() > 0 && (
        m_thruster_vector(m_thruster_vector.size() - 1) != 2 || (m_thruster_vector.size() > 1 
        && m_thruster_vector(m_thruster_vector.size() - 2) != 1))) {
        single_count++;
    }

    int kNumConstraints = 3 * pair_count + single_count; 
    int kNumVariables = m_control_allocation_matrix.cols();
    
    // Initialize the OSQP instance
    osqp::OsqpInstance qp_instance;
    qp_instance.objective_matrix = Q.sparseView();
    qp_instance.objective_vector = c;
    qp_instance.lower_bounds.resize(kNumConstraints);
    qp_instance.upper_bounds.resize(kNumConstraints);

    Eigen::SparseMatrix<double> A_sparse(kNumConstraints, kNumVariables);
    std::vector<Eigen::Triplet<double>> A_triplets;
 
    //Assuming articulated joints are entered first in config file
    // followed by non articulated
    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        int thruster_setting = static_cast<int>(m_thruster_vector[i]);

        switch (thruster_setting) {
            case 0:
                A_triplets.emplace_back((i * 3)/2, i, 1.0);
                break;
            case 1:
                A_triplets.emplace_back((i * 3)/2, i, 1.0);
                A_triplets.emplace_back((i * 3)/2 + 1, i, -omega * deltaT);
                A_triplets.emplace_back((i * 3)/2 + 2, i, -omega * deltaT);
                A_triplets.emplace_back((i * 3)/2 + 1, i + 1, 1.0);
                A_triplets.emplace_back((i * 3)/2+ 2, i + 1, -1.0);
                // @TODO: implementing servo limits
                //A_triplets.emplace_back((i * 3)/2 + 3, i, -min(alpha,gamma+beta));
                //A_triplets.emplace_back((i * 3)/2 + 4, i, max(-alpha,-gamma+beta));
                // A_triplets.emplace_back((i * 3)/2 + 3, i + 1, 1.0);
                // A_triplets.emplace_back((i * 3)/2+ 4, i + 1, -1.0);
                break;
            case 2:
                break;
            default:
                // Handling unexpected settings
                break;
        }
    }

    A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
    qp_instance.constraint_matrix = A_sparse;

    // making this as a helper to build boundaries
    std::vector<int> boundary_slack_vector;

    for (int i = 0; i < m_thruster_vector.size(); ++i) {
        int value = static_cast<int>(m_thruster_vector[i]);
        switch (value) {
            case 1:
                boundary_slack_vector.push_back(1);
                boundary_slack_vector.push_back(1);
                boundary_slack_vector.push_back(1);
                break;
            case 0:
                boundary_slack_vector.push_back(0);
                break;
            case 2:
                // do nothing
                break;
            default:
                // Handle unexpected values
                break;
        }
    }

    for (size_t i = 0; i < boundary_slack_vector.size(); ++i) {
        switch (static_cast<int>(boundary_slack_vector[i])) {
            case 1:
                // F1 and F2 x
                qp_instance.lower_bounds[i] = 0; 
                qp_instance.upper_bounds[i] =  m_upper_limit((i/3)+1) * cos(omega * deltaT);

                for (int j = i + 1; j < i + 3; ++j) {
                    qp_instance.lower_bounds[j] = -kInfinity;
                    qp_instance.upper_bounds[j] = 0;
                }
                i++;
                i++;
                break;
            case 0:
                if ((i + 1) % 3 == 0) {
                    qp_instance.lower_bounds[i] = m_lower_limit[i-2]; 
                    qp_instance.upper_bounds[i] = m_upper_limit[i-2];
                } else { 
                    qp_instance.lower_bounds[i] = m_lower_limit[i-3]; 
                    qp_instance.upper_bounds[i] = m_upper_limit[i-3];
                }
                break;
        }
    }

    // Solve the quadratic program
    osqp::OsqpSolver solver;
    osqp::OsqpSettings settings;
    settings.verbose = false;
    auto status = solver.Init(qp_instance, settings);

    if (!status.ok()) {
        return false;
    }

    osqp::OsqpExitCode exitCode = solver.Solve();

    switch (exitCode) {
        case osqp::OsqpExitCode::kOptimal: {
            *t = solver.primal_solution();

            // Output the optimal solution
            // std::cout << "Optimal solution vector: [" << t->transpose() << "]" << std::endl;
            return true;
        }
        case osqp::OsqpExitCode::kPrimalInfeasible:
            std::cerr << "The problem is primal infeasible." << std::endl;
            break;
        case osqp::OsqpExitCode::kDualInfeasible:
            std::cerr << "The problem is dual infeasible." << std::endl;
            break;
        case osqp::OsqpExitCode::kOptimalInaccurate:
            std::cerr << "The optimal solution is inaccurate." << std::endl;
            break;
        case osqp::OsqpExitCode::kPrimalInfeasibleInaccurate:
            std::cerr << "The problem is primal infeasible and the solution is inaccurate." << std::endl;
            break;
        case osqp::OsqpExitCode::kDualInfeasibleInaccurate:
            std::cerr << "The problem is dual infeasible and the solution is inaccurate." << std::endl;
            break;
        case osqp::OsqpExitCode::kMaxIterations:
            std::cerr << "The maximum number of iterations has been reached." << std::endl;
            break;
        case osqp::OsqpExitCode::kInterrupted:
            std::cerr << "The optimization was interrupted." << std::endl;
            break;
        case osqp::OsqpExitCode::kTimeLimitReached:
            std::cerr << "The time limit was reached before a solution was found." << std::endl;
            break;
        case osqp::OsqpExitCode::kNonConvex:
            std::cerr << "The problem is non-convex." << std::endl;
            break;
        case osqp::OsqpExitCode::kUnknown:
        default:
            std::cerr << "An unknown error occurred." << std::endl;
            break;
    }

    return false;
}

void MvpControl::set_controlled_freedoms(decltype(m_controlled_freedoms) f) {
    m_controlled_freedoms = f;
}

auto MvpControl::get_state_error() -> decltype(this->m_error_state) {
    return m_error_state;
}

Eigen::ArrayXd MvpControl::f_error_function(Eigen::ArrayXd desired,
                                            Eigen::ArrayXd current)
{

    std::lock_guard<std::recursive_mutex> lock(m_desired_state_lock);

    if(desired.size() != current.size()) {
        throw control_exception(
            "desired and current state sizes are different"
        );
    }

    Eigen::ArrayXd error = desired - current;

    for(const auto& i : {DOF::ROLL, DOF::PITCH, DOF::YAW,
                         DOF::ROLL_RATE, DOF::PITCH_RATE, DOF::YAW_RATE}) {

        // todo: wrap2pi implementation

        // auto d = (fmod(desired(i) + M_PI, 2*M_PI) - M_PI);
        // auto c = (fmod(current(i) + M_PI, 2*M_PI) - M_PI);

        // auto t = d - c;
        // double diff = (fmod(t + M_PI, 2*M_PI) - M_PI);
        // error(i) = diff < -M_PI ? diff + 2*M_PI : diff;

        //wrap desired and current in to -pi to pi
        auto d = (fmod(desired(i) + std::copysign(M_PI, desired(i)), 2*M_PI)
                - std::copysign(M_PI, desired(i)));
        auto c = (fmod(current(i) + std::copysign(M_PI,current(i)), 2*M_PI)
                - std::copysign(M_PI,current(i)));
        auto t = d - c;
        double diff = (fmod(t + std::copysign(M_PI,t), 2*M_PI)
                - std::copysign(M_PI,t));
        error(i) = diff;
    }

    m_error_state = error;

    return error;
}

void MvpControl::update_control_allocation_matrix(
        const decltype(m_control_allocation_matrix) &m) {
    std::scoped_lock lock(m_allocation_matrix_lock);
    m_control_allocation_matrix = m;
}



void MvpControl::update_freedoms(std::vector<int> freedoms) {
    std::scoped_lock lock(m_controlled_freedoms_lock);
    m_controlled_freedoms = std::move(freedoms);

}

void MvpControl::update_desired_state(
        const decltype(m_desired_state) &desired_state) {
    std::scoped_lock lock(m_desired_state_lock);
    m_desired_state = desired_state;
}

