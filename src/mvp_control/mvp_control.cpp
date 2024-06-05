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
    std::cout << "Upper limit set to: " << m_upper_limit.transpose() << std::endl;
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

bool MvpControl::f_optimize_thrust(Eigen::VectorXd *t, Eigen::VectorXd u) {
    // Static initialization flag
    static bool is_initialized = false;
    if (!is_initialized) {
        m_current_angles.resize(m_thruster_vector.size(), 0.0);
        is_initialized = true;
    }

    // Control allocation matrix and control vector
    Eigen::MatrixXd T(m_controlled_freedoms.size(), m_control_allocation_matrix.cols());
    Eigen::VectorXd U(m_controlled_freedoms.size());

    {
        // Scoped lock to ensure thread safety
        std::scoped_lock lock(m_allocation_matrix_lock, m_controlled_freedoms_lock);
        for (int i = 0; i < m_controlled_freedoms.size(); ++i) {
            int idx = m_controlled_freedoms.at(i);
            if (idx < 0 || idx >= m_control_allocation_matrix.rows() || idx >= u.size()) {
                ROS_ERROR_STREAM("Index out of bounds when accessing controlled freedoms: " << idx);
                return false;
            }
            T.row(i) = m_control_allocation_matrix.row(idx);
            U(i) = u(idx);
        }
    }

    // Prepare data for the quadratic solver
    Eigen::MatrixXd Q = 2 * T.transpose() * T;
    Eigen::VectorXd c = -2 * T.transpose() * U;

    ROS_INFO_STREAM("m_thruster_vector: " << m_thruster_vector.transpose());

    // Log current angles
    std::stringstream ss;
    ss << "Current Angles vector: ";
    for (const auto &angle : m_current_angles) {
        ss << angle << " ";
    }
    ROS_INFO_STREAM(ss.str());

    // Initialize constraint matrix and bounds
    const double deltaT = 1.0 / m_controller_frequency;

    // Calculate pair and single counts
    for (size_t i = 0; i + 1 < m_thruster_vector.size(); ++i) {
        if (m_thruster_vector(i) == 1 && m_thruster_vector(i + 1) == 2) {
            pair_count++;
            i++;
        } else {
            single_count++;
        }
    }

    // Check the last element if it's not part of the last checked pair
    if (m_thruster_vector.size() > 0 && 
        (m_thruster_vector(m_thruster_vector.size() - 1) != 2 || 
         (m_thruster_vector.size() > 1 && m_thruster_vector(m_thruster_vector.size() - 2) != 1))) {
        single_count++;
    }

    int kNumConstraints = 3 * pair_count + single_count;
    int kNumVariables = m_control_allocation_matrix.cols();

    //helper for adjusted dimension boundries vector
    std::vector<int> thruster_case_values;
    thruster_case_values.reserve(kNumConstraints);

    // Populate thruster case values vector and adjusted upper and lower limit vectors
    m_adjusted_upper_limit.clear();
    m_adjusted_lower_limit.clear();
    thruster_case_values.clear();

    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        int thruster_value = m_thruster_vector[i];
        thruster_case_values.push_back(thruster_value == 2 ? 1 : thruster_value);

        m_adjusted_upper_limit.push_back(m_upper_limit[i]);
        m_adjusted_lower_limit.push_back(m_lower_limit[i]);

        if (thruster_value == 2) {
            m_adjusted_upper_limit.push_back(m_upper_limit[i]);
            m_adjusted_lower_limit.push_back(m_lower_limit[i]);
        }
    }

    osqp::OsqpInstance qp_instance;
    qp_instance.objective_matrix = Q.sparseView();
    qp_instance.objective_vector = c;
    qp_instance.lower_bounds.resize(kNumConstraints);
    qp_instance.upper_bounds.resize(kNumConstraints);

    Eigen::SparseMatrix<double> A_sparse(kNumConstraints, kNumVariables);
    A_sparse.setZero(); //all constraint matrix values set to 0
    std::vector<Eigen::Triplet<double>> A_triplets;

    const double omega_deltaT = omega * deltaT;
    size_t j = 0;  //row  counter

    for (size_t i = 0; i < kNumConstraints; ++i) {
        int thruster_setting = static_cast<int>(m_thruster_vector[i]);
        double beta = m_current_angles[i];
        switch (thruster_setting) {
            case 0:
                A_triplets.emplace_back(j, i, 1.0);
                qp_instance.lower_bounds[j] = m_adjusted_lower_limit[j];
                qp_instance.upper_bounds[j] = m_adjusted_upper_limit[j];
                j++;
                break;
            case 1:
                A_triplets.emplace_back(j, i, 1.0);
                A_triplets.emplace_back(j + 1, i, tan(-std::min(omega_deltaT, gamma_upper - beta)));
                A_triplets.emplace_back(j + 2, i, tan(std::max(-omega_deltaT, gamma_lower - beta)));
                A_triplets.emplace_back(j + 1, i + 1, 1.0);
                A_triplets.emplace_back(j + 2, i + 1, -1.0);

                qp_instance.lower_bounds[j] = 0;
                qp_instance.upper_bounds[j] = m_adjusted_upper_limit[j] * std::cos(omega_deltaT);
                qp_instance.lower_bounds[j + 1] = -kInfinity;
                qp_instance.upper_bounds[j + 1] = 0;
                qp_instance.lower_bounds[j + 2] = -kInfinity;
                qp_instance.upper_bounds[j + 2] = 0;
                j += 3;
                break;
            case 2:
                break;
            default:
                ROS_ERROR_STREAM("Unexpected thruster setting: " << thruster_setting);
                return false;
        }
    }

    A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
    qp_instance.constraint_matrix = A_sparse;

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
        case osqp::OsqpExitCode::kOptimal:
            *t = solver.primal_solution();
            return true;
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

