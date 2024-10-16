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

    Author: Farhang Naderi
    Email: farhang.naderi@uri.edu;farhang.nba@gmail.com
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
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
}

void MvpControl::set_upper_limit(const decltype(m_upper_limit) &upper_limit) {
    m_upper_limit = upper_limit;
}

void MvpControl::set_lower_angle(const decltype(m_lower_angle) &lower_angle) {
    m_lower_angle = lower_angle;
}

void MvpControl::set_upper_angle(const decltype(m_upper_angle) &upper_angle) {
    m_upper_angle = upper_angle;
}

void MvpControl::set_servo_speed(const decltype(m_servo_speed) &servo_speed) {
    m_servo_speed = servo_speed;
}

void MvpControl::set_current_angle(const int* m_thruster_index, double angle) {
    m_current_angles[*m_thruster_index] = angle;
}

// Setter for thruster directions
void MvpControl::set_thrust_direction(const std::vector<int>& thruster_directions) {
    // if (thruster_directions.size() == m_thruster_directions.size()) {
        m_thruster_directions = thruster_directions;  // Update if sizes match
    // } else {
    //     ROS_WARN("Size mismatch in thruster directions. Expected %zu, got %zu", m_thruster_directions.size(), thruster_directions.size());
    // }
}


// Getter for thruster directions
std::vector<int> MvpControl::get_thrust_direction() const {
    return m_thruster_directions;  // Return the stored thruster directions
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

// bool MvpControl::f_optimize_thrust(Eigen::VectorXd *t, Eigen::VectorXd u) {
//     static bool is_initialized = false;
//     if (!is_initialized) {
//         m_current_angles.resize(m_thruster_vector.size(), 0.0);
//         is_initialized = true;
//     }

//     // Allocate and initialize control matrices and vectors
//     Eigen::MatrixXd T(m_controlled_freedoms.size(), m_control_allocation_matrix.cols());
//     Eigen::VectorXd U(m_controlled_freedoms.size());

//     // Scoped lock for thread safety when accessing shared resources
//     {
//         std::scoped_lock lock(m_allocation_matrix_lock, m_controlled_freedoms_lock);
//         for (int i = 0; i < m_controlled_freedoms.size(); ++i) {
//             int idx = m_controlled_freedoms.at(i);
//             if (idx < 0 || idx >= m_control_allocation_matrix.rows() || idx >= u.size()) {
//                 ROS_ERROR_STREAM("Index out of bounds when accessing controlled freedoms: " << idx);
//                 return false;
//             }
//             T.row(i) = m_control_allocation_matrix.row(idx);
//             U(i) = u(idx);
//         }
//     }

//     // Prepare data for the quadratic solver
//     Eigen::MatrixXd Q = 2 * T.transpose() * T;
//     Eigen::VectorXd c = -2 * T.transpose() * U;

//     double deltaT = 1.0 / m_controller_frequency;

//     // Calculate the number of pairs and singles in the thruster vector
//     auto [pair_count, single_count] = [this] {
//         int pairs = 0;
//         int singles = 0;
//         for (int i = 0; i + 1 < m_thruster_vector.size(); ++i) {
//             if (m_thruster_vector(i) == 1 && m_thruster_vector(i + 1) == 2) {
//                 pairs++;
//                 i++; // Skip the next element since it forms a pair with the current element
//             } else {
//                 singles++;
//             }
//         }
//         return std::make_pair(pairs, singles);
//     }();

//     // Check the last element if it's not part of the last checked pair
//     if (m_thruster_vector.size() > 0 && 
//         (m_thruster_vector(m_thruster_vector.size() - 1) != 2 || 
//          (m_thruster_vector.size() > 1 && m_thruster_vector(m_thruster_vector.size() - 2) != 1))) {
//         single_count++;
//     }

//     int kNumConstraints = 5 * pair_count + single_count;
//     int kNumVariables = m_control_allocation_matrix.cols();

//     // Helper for adjusted dimension boundaries vector
//     std::vector<int> thruster_case_values;
//     thruster_case_values.reserve(kNumConstraints);

//     // Initialize thrusters direction vector with zeros 
//     std::vector<int> thruster_direction_action(m_thruster_vector.size(), 1);


//     // Retrieve the thruster directions using the getter
//     thruster_direction_action = get_thrust_direction();
//     //ROS_INFO("Thruster direction action size: %zu", thruster_direction_action.size());

//     if (thruster_direction_action.size() != m_thruster_vector.size()) {
//       //  ROS_WARN("Thruster direction action size does not match m_thruster_vector size. Setting default values.");
//         thruster_direction_action.resize(m_thruster_vector.size(), 1); // Resize and set to 1 if needed
//     }

//     // Ensure that each direction is valid, or default to 1
//     for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
//         // If no valid data is received (not 1 or -1), set the default value to 1
//         if (thruster_direction_action[i] != 1 && thruster_direction_action[i] != -1) {
//             thruster_direction_action[i] = 1;  // Default value for invalid data
//         } else {
//             //ROS_INFO("Thruster %zu direction is valid: %d", i, thruster_direction_action[i]);
//         }
//     }
    
//     // Populate thruster case values vector and adjusted upper and lower limit vectors
//     m_adjusted_upper_limit.clear();
//     m_adjusted_lower_limit.clear();
//     thruster_case_values.clear();

//     //The idea here is top make bounds columns same as A sparse matrix for OSQP
//     for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
//         int thruster_value = m_thruster_vector[i];

//         // Adjust thruster case value (treat value 2 as 1)
//         if (thruster_value == 0)
//         {
//             thruster_case_values.push_back(0);  // Add 0 if thruster_value is 0
//             m_adjusted_upper_limit.push_back(m_upper_limit[i]);
//             m_adjusted_lower_limit.push_back(m_lower_limit[i]);
//         }
//         else if (thruster_value == 2)
//         {
//             thruster_case_values.push_back(1);  // Add 1 if thruster_value is 2
//             m_adjusted_upper_limit.push_back(m_upper_limit[i]);
//             m_adjusted_lower_limit.push_back(m_lower_limit[i]);
//             m_adjusted_upper_limit.push_back(m_upper_limit[i]);
//             m_adjusted_lower_limit.push_back(m_lower_limit[i]);
//         }
//         else
//         {
//             thruster_case_values.push_back(thruster_value);  // Otherwise, add thruster_value itself
//             m_adjusted_upper_limit.push_back(m_upper_limit[i]);
//             m_adjusted_lower_limit.push_back(m_lower_limit[i]);
//             m_adjusted_upper_limit.push_back(m_upper_limit[i]);
//             m_adjusted_lower_limit.push_back(m_lower_limit[i]);
//             m_adjusted_upper_limit.push_back(m_upper_limit[i]);
//             m_adjusted_lower_limit.push_back(m_lower_limit[i]);
//         }
//         // Add the upper and lower limits
//        // m_adjusted_upper_limit.push_back(m_upper_limit[i]);
//        // m_adjusted_lower_limit.push_back(m_lower_limit[i]);

//         // If thruster value is 2, add the limits two more times for the paired thruster
//         // if (thruster_value == 2) {
//             // for (int j = 0; j < 2; ++j) {


//             // }

//         // }
//     }
//     // std::cout << "Size of m_adjusted_lower_limit: " << m_adjusted_lower_limit.size() << std::endl;
//     // std::cout << "Size of m_adjusted_upper_limit: " << m_adjusted_upper_limit.size() << std::endl;


//     //     // Print m_adjusted_lower_limit
//     //     std::cout << "m_adjusted_lower_limit: ";
//     //     for (size_t i = 0; i < m_adjusted_lower_limit.size(); ++i) {
//     //         std::cout << m_adjusted_lower_limit[i] << " ";
//     //     }
//     //     std::cout << std::endl; // New line for clarity

//     //     // Print m_adjusted_upper_limit
//     //     std::cout << "m_adjusted_upper_limit: ";
//     //     for (size_t i = 0; i < m_adjusted_upper_limit.size(); ++i) {
//     //         std::cout << m_adjusted_upper_limit[i] << " ";
//     //     }
//     //     std::cout << std::endl; // New line for clarity

//     // Setup OSQP solver instance
//     osqp::OsqpInstance qp_instance;
//     qp_instance.objective_matrix = Q.sparseView();
//     qp_instance.objective_vector = c;
//     qp_instance.lower_bounds.resize(kNumConstraints);
//     qp_instance.upper_bounds.resize(kNumConstraints);

//     Eigen::SparseMatrix<double> A_sparse(kNumConstraints, kNumVariables);
//     A_sparse.setZero(); // Set all constraint matrix values to 0
//     std::vector<Eigen::Triplet<double>> A_triplets;

//     int j = 0;  // Row counter
//     const double PI_VALUE = 3.14159265358979323846;

//     // Construct constraint matrix and bounds
//     for (int i = 0; i < m_thruster_vector.size(); ++i) {
//         int thruster_setting = static_cast<int>(m_thruster_vector[i]);



//         switch (thruster_setting) {
//             case 0:
//                 A_triplets.emplace_back(j, i, 1.0);
//                 // Before setting bounds, log the current stats
//                 //printf("Setting bounds at index %d: lower = %d, upper = %d\n", j, m_adjusted_lower_limit[j], m_adjusted_upper_limit[j]);
//                 qp_instance.lower_bounds[j] = m_adjusted_lower_limit[j];
//                 qp_instance.upper_bounds[j] = m_adjusted_upper_limit[j];


//                 j++;
//                 //printf("j after non-articulated: %d\n", j);
//                 break;
//             case 1:

//                 // A_triplets.emplace_back(j, i, 1.0);
//                 // A_triplets.emplace_back(j + 1, i, tan(-std::min(m_servo_speed[i] * deltaT , m_upper_angle[i] - beta)));
//                 // A_triplets.emplace_back(j + 2, i, tan(std::max(-m_servo_speed[i] * deltaT , m_lower_angle[i] - beta)));
//                 // A_triplets.emplace_back(j + 1, i + 1, 1.0);
//                 // A_triplets.emplace_back(j + 2, i + 1, -1.0);

//                 // qp_instance.lower_bounds[j] = 0;
//                 // qp_instance.upper_bounds[j] = m_adjusted_upper_limit[j] * std::cos(m_servo_speed[i] * deltaT);
//                 // qp_instance.lower_bounds[j + 1] = -kInfinity;
//                 // qp_instance.upper_bounds[j + 1] = 0;
//                 // qp_instance.lower_bounds[j + 2] = -kInfinity;
//                 // qp_instance.upper_bounds[j + 2] = 0;
//                 // j += 3; //jumping the constraint rows
//                 // break;
//                 if (thruster_direction_action[i] == 1)
//                 {
//                 //const double PI_VALUE = 3.14159265358979323846;
//                 // A_triplets.emplace_back(j, i, 1.0);
//                 // A_triplets.emplace_back(j + 1, i, -tan(m_servo_speed[i] * deltaT ));
//                 // A_triplets.emplace_back(j + 2, i, -tan(-m_servo_speed[i] * deltaT ));
//                 // // A_triplets.emplace_back(j + 3, i, -tan(m_upper_angle[i] -  m_current_angles[i]   ));
//                 // // A_triplets.emplace_back(j + 4, i, -tan(m_lower_angle[i] -  m_current_angles[i]   ));
//                 // A_triplets.emplace_back(j + 3, i, -tan(fmod(m_upper_angle[i] - m_current_angles[i] + M_PI, 2 * M_PI) - M_PI));
//                 // A_triplets.emplace_back(j + 4, i, -tan(fmod(m_lower_angle[i] - m_current_angles[i] + M_PI, 2 * M_PI) - M_PI));  
//                 // //printf("Difference Angles: %f\n", -tan(fmod(m_lower_angle[i] - m_current_angles[i] + M_PI, 2 * M_PI) - M_PI));
//                 // A_triplets.emplace_back(j + 1, i + 1, 1.0);
//                 // A_triplets.emplace_back(j + 2, i + 1, 1.0);
//                 // A_triplets.emplace_back(j + 3, i + 1, 1.0);
//                 // A_triplets.emplace_back(j + 4, i + 1, 1.0);

//                 // qp_instance.lower_bounds[j] =      0.0;
//                 // qp_instance.upper_bounds[j] =     30.0; // m_adjusted_upper_limit[j] * std::cos(m_servo_speed[i] * deltaT);
//                 // qp_instance.lower_bounds[j + 1] = -1e10;
//                 // qp_instance.upper_bounds[j + 1] =  0.0;
//                 // qp_instance.lower_bounds[j + 2] =  0.0;
//                 // qp_instance.upper_bounds[j + 2] =  1e10;
//                 // qp_instance.lower_bounds[j + 3] = -1e10;
//                 // qp_instance.upper_bounds[j + 3] =  0.0;
//                 // qp_instance.lower_bounds[j + 4] =  0.0;
//                 // qp_instance.upper_bounds[j + 4] =  1e10;

//                 // Compute wrapped angle differences using PI_VALUE
//                 double angle_diff_upper = std::fmod(m_upper_angle[i] - m_current_angles[i] + PI_VALUE, 2 * PI_VALUE) - PI_VALUE;
//                 double angle_diff_lower = std::fmod(m_lower_angle[i] - m_current_angles[i] + PI_VALUE, 2 * PI_VALUE) - PI_VALUE;

//                 // Use angle differences directly (linear approximation)
//                 A_triplets.emplace_back(j, i, 1.0);
//                 A_triplets.emplace_back(j + 1, i, -m_servo_speed[i] * deltaT);
//                 A_triplets.emplace_back(j + 2, i, m_servo_speed[i] * deltaT);
//                 A_triplets.emplace_back(j + 3, i, -angle_diff_upper);
//                 A_triplets.emplace_back(j + 4, i, -angle_diff_lower);

//                 // Debugging output for angle differences
//                 // printf("Difference Angles Upper (Joint %zu): %f\n", i, angle_diff_upper);
//                 // printf("Difference Angles Lower (Joint %zu): %f\n", i, angle_diff_lower);

//                 // Set up the rest of the matrix
//                 A_triplets.emplace_back(j + 1, i + 1, 1.0);
//                 A_triplets.emplace_back(j + 2, i + 1, 1.0);
//                 A_triplets.emplace_back(j + 3, i + 1, 1.0);
//                 A_triplets.emplace_back(j + 4, i + 1, 1.0);

//                 // Set up the bounds for the optimization problem
//                 qp_instance.lower_bounds[j]     = 0.0;
//                 qp_instance.upper_bounds[j]     = m_adjusted_upper_limit[j];  
//                 qp_instance.lower_bounds[j + 1] = -kInfinity;
//                 qp_instance.upper_bounds[j + 1] = 0.0;
//                 qp_instance.lower_bounds[j + 2] = 0.0;
//                 qp_instance.upper_bounds[j + 2] = kInfinity;
//                 qp_instance.lower_bounds[j + 3] = -kInfinity;
//                 qp_instance.upper_bounds[j + 3] = 0.0;
//                 qp_instance.lower_bounds[j + 4] = 0.0;
//                 qp_instance.upper_bounds[j + 4] = kInfinity;
//                 j += 5; //jumping the constraint rows

//                 // A_triplets.emplace_back(j, i, 1.0);
//                 // A_triplets.emplace_back(j + 1, i, tan(-std::min(m_servo_speed[i] * deltaT , m_upper_angle[i] - m_current_angles[i])));
//                 // A_triplets.emplace_back(j + 2, i, tan(std::max(-m_servo_speed[i] * deltaT , m_lower_angle[i] - m_current_angles[i])));
//                 // A_triplets.emplace_back(j + 1, i + 1, 1.0);
//                 // A_triplets.emplace_back(j + 2, i + 1, -1.0);


//                 // qp_instance.lower_bounds[j] = 0;
//                 // qp_instance.upper_bounds[j] = m_adjusted_upper_limit[j] * std::cos(m_servo_speed[i] * deltaT);
//                 // qp_instance.lower_bounds[j + 1] = -kInfinity;
//                 // qp_instance.upper_bounds[j + 1] = 0;
//                 // qp_instance.lower_bounds[j + 2] = -kInfinity;
//                 // qp_instance.upper_bounds[j + 2] = 0;
//                 // j += 3; //jumping the constraint rows
//                 break;

//                 }
//                 else if (thruster_direction_action[i] == -1){
//                 // //ROS_INFO("m_current_angles[%d]: %f", i, m_current_angles[i]);
//                 // A_triplets.emplace_back(j, i, 1.0);
//                 // A_triplets.emplace_back(j + 1, i, -tan(m_servo_speed[i] * deltaT ));
//                 // A_triplets.emplace_back(j + 2, i, -tan(-m_servo_speed[i] * deltaT ));
//                 // A_triplets.emplace_back(j + 3, i, -tan(m_upper_angle[i] - m_current_angles[i]   ));
//                 // A_triplets.emplace_back(j + 4, i, -tan(m_lower_angle[i] - m_current_angles[i]   ));
//                 // A_triplets.emplace_back(j + 1, i + 1, 1.0);
//                 // A_triplets.emplace_back(j + 2, i + 1, 1.0);
//                 // A_triplets.emplace_back(j + 3, i + 1, 1.0);
//                 // A_triplets.emplace_back(j + 4, i + 1, 1.0);

//                 // qp_instance.lower_bounds[j] = -m_adjusted_lower_limit[j] * abs(std::cos(m_servo_speed[i] * deltaT));
//                 // qp_instance.upper_bounds[j] = 0;
//                 // qp_instance.lower_bounds[j + 1] = 0;
//                 // qp_instance.upper_bounds[j + 1] = kInfinity;
//                 // qp_instance.lower_bounds[j + 2] = -kInfinity;
//                 // qp_instance.upper_bounds[j + 2] = 0;
//                 // qp_instance.lower_bounds[j + 3] = 0;
//                 // qp_instance.upper_bounds[j + 3] = kInfinity;
//                 // qp_instance.lower_bounds[j + 4] = -kInfinity;
//                 // qp_instance.upper_bounds[j + 4] = 0;
//                 // j += 5; //jumping the constraint rows

//                 // Define PI constant to avoid conflicts


//                 // Compute angle differences and wrap them to [-PI, PI]
//                 double angle_diff_upper = std::fmod(m_upper_angle[i] - m_current_angles[i] + PI_VALUE, 2 * PI_VALUE) - PI_VALUE;
//                 double angle_diff_lower = std::fmod(m_lower_angle[i] - m_current_angles[i] + PI_VALUE, 2 * PI_VALUE) - PI_VALUE;

//                 // Update A_triplets using angle differences directly
//                 A_triplets.emplace_back(j, i, 1.0);
//                 A_triplets.emplace_back(j + 1, i, -m_servo_speed[i] * deltaT);                // tan(x) ≈ x
//                 A_triplets.emplace_back(j + 2, i, m_servo_speed[i] * deltaT);                 // tan(-x) ≈ -x
//                 A_triplets.emplace_back(j + 3, i, -angle_diff_upper);
//                 A_triplets.emplace_back(j + 4, i, -angle_diff_lower);

//                 A_triplets.emplace_back(j + 1, i + 1, 1.0);
//                 A_triplets.emplace_back(j + 2, i + 1, 1.0);
//                 A_triplets.emplace_back(j + 3, i + 1, 1.0);
//                 A_triplets.emplace_back(j + 4, i + 1, 1.0);

//                 // Update qp_instance bounds
//                 qp_instance.lower_bounds[j]     = m_adjusted_lower_limit[j]; 
//                 qp_instance.upper_bounds[j]     = 0.0;
//                 qp_instance.lower_bounds[j + 1] = 0.0;
//                 qp_instance.upper_bounds[j + 1] = kInfinity;
//                 qp_instance.lower_bounds[j + 2] = -kInfinity;
//                 qp_instance.upper_bounds[j + 2] = 0.0;
//                 qp_instance.lower_bounds[j + 3] = 0.0;
//                 qp_instance.upper_bounds[j + 3] = kInfinity;
//                 qp_instance.lower_bounds[j + 4] = -kInfinity;
//                 qp_instance.upper_bounds[j + 4] = 0.0;

//                 j += 5; // jumping the constraint rows

//                 // printf("Angle Increment (Joint %zu): %f\n", i, m_servo_speed[i] * deltaT);
//                 // printf("Difference Angles Upper (Joint %zu): %f\n", i, angle_diff_upper);
//                 // printf("Difference Angles Lower (Joint %zu): %f\n", i, angle_diff_lower);
//                 // A_triplets.emplace_back(j, i, 1.0);
//                 // A_triplets.emplace_back(j + 1, i, tan(std::max(m_servo_speed[i] * deltaT , m_current_angles[i] - m_upper_angle[i])));
//                 // A_triplets.emplace_back(j + 2, i, tan(-std::min(m_servo_speed[i] * deltaT , m_current_angles[i] - m_lower_angle[i])));
//                 // A_triplets.emplace_back(j + 1, i + 1, 1.0);
//                 // A_triplets.emplace_back(j + 2, i + 1, -1.0);
//                 // //printf("m_current_angles[%d]: %f\n", i, m_current_angles[i]);
//                 // qp_instance.lower_bounds[j] = m_adjusted_lower_limit[j] * std::cos(m_servo_speed[i] * deltaT);
//                 // qp_instance.upper_bounds[j] = 0;
//                 // qp_instance.lower_bounds[j + 1] = -kInfinity;
//                 // qp_instance.upper_bounds[j + 1] = 0;
//                 // qp_instance.lower_bounds[j + 2] = -kInfinity;
//                 // qp_instance.upper_bounds[j + 2] = 0;
//                 // j += 3; //jumping the constraint rows
//                 break;

//                 }
//                 else
//                 {
//                     // ROS_ERROR("Thruster direction is not set!");
//                     // return false;
//                     continue;
//                 }
//             case 2:
//                 // Add any specific handling for thruster setting 2 if necessary
//                 break;
//             default:
//                 ROS_ERROR_STREAM("Unexpected thruster setting: " << thruster_setting);
//                 return false;
//         }
//     }

//     // Populate constraint matrix
//     A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
//     qp_instance.constraint_matrix = A_sparse;
//     // Initialize OSQP solver
//     osqp::OsqpSolver solver;
//     osqp::OsqpSettings settings;
//     settings.verbose = false;
//     auto status = solver.Init(qp_instance, settings);

//     // for (int k = 0; k < A_sparse.outerSize(); ++k) {
//     //     for (Eigen::SparseMatrix<double>::InnerIterator it(A_sparse, k); it; ++it) {
//     //         std::cout << "A(" << it.row() << ", " << it.col() << ") = " << it.value() << std::endl;
//     //     }
//     // }

//     if (!status.ok()) {
//         ROS_ERROR("OSQP solver initialization failed.");
//         return false;
//     }


//     // Solve the quadratic programming problem
//     osqp::OsqpExitCode exitCode = solver.Solve();

//     // Handle solver exit codes
//     switch (exitCode) {
//         case osqp::OsqpExitCode::kOptimal:
//             *t = solver.primal_solution();
//             return true;
//         case osqp::OsqpExitCode::kPrimalInfeasible:
//             ROS_ERROR("The problem is primal infeasible.");
//             break;
//         case osqp::OsqpExitCode::kDualInfeasible:
//             ROS_ERROR("The problem is dual infeasible.");
//             break;
//         case osqp::OsqpExitCode::kOptimalInaccurate:
//             ROS_ERROR("The optimal solution is inaccurate.");
//             break;
//         case osqp::OsqpExitCode::kPrimalInfeasibleInaccurate:
//             ROS_ERROR("The problem is primal infeasible and the solution is inaccurate.");
//             break;
//         case osqp::OsqpExitCode::kDualInfeasibleInaccurate:
//             ROS_ERROR("The problem is dual infeasible and the solution is inaccurate.");
//             break;
//         case osqp::OsqpExitCode::kMaxIterations:
//             ROS_ERROR("The maximum number of iterations has been reached.");
//             break;
//         case osqp::OsqpExitCode::kInterrupted:
//             ROS_ERROR("The optimization was interrupted.");
//             break;
//         case osqp::OsqpExitCode::kTimeLimitReached:
//             ROS_ERROR("The time limit was reached before a solution was found.");
//             break;
//         case osqp::OsqpExitCode::kNonConvex:
//             ROS_ERROR("The problem is non-convex.");
//             break;
//         case osqp::OsqpExitCode::kUnknown:
//         default:
//             ROS_ERROR("An unknown error occurred.");
//             break;
//     }

//     return false;
// }


bool MvpControl::f_optimize_thrust(Eigen::VectorXd *t, Eigen::VectorXd u) {
    // static bool is_initialized = false;
    // if (!is_initialized) {
        // Initialize current angles if not already done
        //m_current_angles.resize(m_thruster_vector.size(), 0.0);
        m_current_angles.resize(m_thruster_vector.size());
        std::vector<int> thruster_direction_action = get_thrust_direction();
        // printf("thruster_direction_action size: %zu\n", thruster_direction_action.size());
        // is_initialized = true;
    // }

    // Static variable to store the previous thruster directions
    static std::vector<int> prev_thruster_direction_action;

    // Detect changes in thruster directions
    bool direction_changed = false;
    if (prev_thruster_direction_action.empty()) {
        // First call, initialize the previous directions
        prev_thruster_direction_action = thruster_direction_action;
    } else if (prev_thruster_direction_action.size() != thruster_direction_action.size()) {
        // Size changed, so direction has changed
        direction_changed = true;
    } else {
        // Compare each element
        for (size_t i = 0; i < thruster_direction_action.size(); ++i) {
            if (thruster_direction_action[i] != prev_thruster_direction_action[i]) {
                direction_changed = true;
                break;
            }
        }
    }

    if (direction_changed) {
        // Pause for 2.0 seconds
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // Set the solution vector to zero
        *t = Eigen::VectorXd::Zero(m_thruster_vector.size());
        // Update the previous thruster directions
        prev_thruster_direction_action = thruster_direction_action;
        // Return true since we've provided a valid solution
        return true;
    }

    // Update the previous thruster directions
    prev_thruster_direction_action = thruster_direction_action;

    // Allocate and initialize control matrices and vectors
    Eigen::MatrixXd T(m_controlled_freedoms.size(), m_control_allocation_matrix.cols());
    Eigen::VectorXd U(m_controlled_freedoms.size());

    // Scoped lock for thread safety when accessing shared resources
    {
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

    double deltaT = 1.0 / m_controller_frequency;

    // Calculate the number of pairs and singles in the thruster vector
    int pair_count = 0;
    int single_count = 0;
    for (size_t i = 0; i + 1 < m_thruster_vector.size(); ++i) {
        if (m_thruster_vector(i) == 1 && m_thruster_vector(i + 1) == 2) {
            pair_count++;
            i++; // Skip the next element since it forms a pair with the current element
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

    int kNumConstraints = 5 * pair_count + single_count;
    int kNumVariables = m_control_allocation_matrix.cols();

    // Initialize thruster direction vector

    //std::vector<int> thruster_direction_action = get_thrust_direction();

    if (thruster_direction_action.size() != m_thruster_vector.size()) {
        thruster_direction_action.resize(m_thruster_vector.size(), 1); // Default to 1
    }
    // Ensure that each direction is valid, or default to 1
    // for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
    //     if (thruster_direction_action[i] != 1 && thruster_direction_action[i] != -1) {
    //         thruster_direction_action[i] = 1;
    //     }
    // }

   // Step 1: Calculate the total number of elements needed
    int total_elements = 0;
    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        int thruster_value = m_thruster_vector[i];
        if (thruster_value == 0) {
            total_elements += 1;
        }
        else if (thruster_value == 2) {
            total_elements += 2;
        }
        else {
            total_elements += 3;
        }
    }

    // Step 2: Initialize Eigen vectors with the calculated size
    Eigen::VectorXi m_adjusted_upper_limit(total_elements);
    Eigen::VectorXi m_adjusted_lower_limit(total_elements);

    // Step 3: Populate the Eigen vectors
    int current_index = 0;
    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        int thruster_value = m_thruster_vector[i];
        int upper = m_upper_limit[i];
        int lower = m_lower_limit[i];

        if (thruster_value == 0) {
            // Non-articulated thruster
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
        }
        else if (thruster_value == 2) {
            // Articulated thruster (value 2 treated as 1)
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
        }
        else {
            // Other articulated thruster
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
            m_adjusted_upper_limit(current_index) = upper;
            m_adjusted_lower_limit(current_index) = lower;
            current_index++;
        }
    }

    // // Optional: Print the results for verification
    // std::cout << "Adjusted Upper Limits:\n" << m_adjusted_upper_limit << "\n\n";
    // std::cout << "Adjusted Lower Limits:\n" << m_adjusted_lower_limit << "\n";

    // Setup OSQP solver instance
    osqp::OsqpInstance qp_instance;
    qp_instance.objective_matrix = Q.sparseView();
    qp_instance.objective_vector = c;
    qp_instance.lower_bounds.resize(kNumConstraints);
    qp_instance.upper_bounds.resize(kNumConstraints);

    Eigen::SparseMatrix<double> A_sparse(kNumConstraints, kNumVariables);
    A_sparse.setZero(); // Set all constraint matrix values to 0
    std::vector<Eigen::Triplet<double>> A_triplets;

    int j = 0;  // Row counter
    const double PI_VALUE = 3.14159265358979323846;
    const double kInfinity = std::numeric_limits<double>::infinity();

    // Construct constraint matrix and bounds
    for (size_t i = 0; i < m_thruster_vector.size(); ++i) {
        int thruster_setting = static_cast<int>(m_thruster_vector[i]);

        switch (thruster_setting) {
            case 0:
                // Non-articulated thruster constraints
                A_triplets.emplace_back(j, i, 1.0);
                qp_instance.lower_bounds[j] = m_adjusted_lower_limit[j];
                qp_instance.upper_bounds[j] = m_adjusted_upper_limit[j];
                j++;
                break;

            case 1:
                // Articulated thruster constraints
                if (thruster_direction_action[i] == 1) {
                    // Positive thrust direction
                    double angle_diff_upper = std::fmod(m_upper_angle[i] - m_current_angles[i] + PI_VALUE, 2 * PI_VALUE) - PI_VALUE;
                    double angle_diff_lower = std::fmod(m_lower_angle[i] - m_current_angles[i] + PI_VALUE, 2 * PI_VALUE) - PI_VALUE;

                    A_triplets.emplace_back(j, i, 1.0);
                    A_triplets.emplace_back(j + 1, i, -m_servo_speed[i] * deltaT);
                    A_triplets.emplace_back(j + 2, i, m_servo_speed[i] * deltaT);
                    A_triplets.emplace_back(j + 3, i, -angle_diff_upper);
                    A_triplets.emplace_back(j + 4, i, -angle_diff_lower);

                    A_triplets.emplace_back(j + 1, i + 1, 1.0);
                    A_triplets.emplace_back(j + 2, i + 1, 1.0);
                    A_triplets.emplace_back(j + 3, i + 1, 1.0);
                    A_triplets.emplace_back(j + 4, i + 1, 1.0);

                    qp_instance.lower_bounds[j]     = 0.0;
                    qp_instance.upper_bounds[j]     = m_adjusted_upper_limit[j];  
                    qp_instance.lower_bounds[j + 1] = -kInfinity;
                    qp_instance.upper_bounds[j + 1] = 0.0;
                    qp_instance.lower_bounds[j + 2] = 0.0;
                    qp_instance.upper_bounds[j + 2] = kInfinity;
                    qp_instance.lower_bounds[j + 3] = -kInfinity;
                    qp_instance.upper_bounds[j + 3] = 0.0;
                    qp_instance.lower_bounds[j + 4] = 0.0;
                    qp_instance.upper_bounds[j + 4] = kInfinity;
                    j += 5;
                } else if (thruster_direction_action[i] == -1) {
                    // Negative thrust direction
                    double angle_diff_upper = std::fmod(m_upper_angle[i] - m_current_angles[i] + PI_VALUE, 2 * PI_VALUE) - PI_VALUE;
                    double angle_diff_lower = std::fmod(m_lower_angle[i] - m_current_angles[i] + PI_VALUE, 2 * PI_VALUE) - PI_VALUE;

                    A_triplets.emplace_back(j, i, 1.0);
                    A_triplets.emplace_back(j + 1, i, -m_servo_speed[i] * deltaT);
                    A_triplets.emplace_back(j + 2, i, m_servo_speed[i] * deltaT);
                    A_triplets.emplace_back(j + 3, i, -angle_diff_upper);
                    A_triplets.emplace_back(j + 4, i, -angle_diff_lower);

                    A_triplets.emplace_back(j + 1, i + 1, 1.0);
                    A_triplets.emplace_back(j + 2, i + 1, 1.0);
                    A_triplets.emplace_back(j + 3, i + 1, 1.0);
                    A_triplets.emplace_back(j + 4, i + 1, 1.0);

                    qp_instance.lower_bounds[j]     = m_adjusted_lower_limit[j]; 
                    qp_instance.upper_bounds[j]     = 0.0;
                    qp_instance.lower_bounds[j + 1] = 0.0;
                    qp_instance.upper_bounds[j + 1] = kInfinity;
                    qp_instance.lower_bounds[j + 2] = -kInfinity;
                    qp_instance.upper_bounds[j + 2] = 0.0;
                    qp_instance.lower_bounds[j + 3] = 0.0;
                    qp_instance.upper_bounds[j + 3] = kInfinity;
                    qp_instance.lower_bounds[j + 4] = -kInfinity;
                    qp_instance.upper_bounds[j + 4] = 0.0;
                    j += 5;
                } else {
                    ROS_ERROR("Thruster direction is not set!");
                    return false;
                }
                break;

            case 2:
                // Specific handling for thruster setting 2 if necessary
                break;

            default:
                ROS_ERROR_STREAM("Unexpected thruster setting: " << thruster_setting);
                return false;
        }
    }

    // Populate constraint matrix
    A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());
    qp_instance.constraint_matrix = A_sparse;

    // Initialize OSQP solver
    osqp::OsqpSolver solver;
    osqp::OsqpSettings settings;
    settings.verbose = false;
    auto status = solver.Init(qp_instance, settings);

    if (!status.ok()) {
        ROS_ERROR("OSQP solver initialization failed.");
        return false;
    }

    // Solve the quadratic programming problem
    osqp::OsqpExitCode exitCode = solver.Solve();

    // Handle solver exit codes
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