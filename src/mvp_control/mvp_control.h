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


#pragma once

/*******************************************************************************
 * Eigen
 */
#include "Eigen/Dense"

/*******************************************************************************
 * Quadratic Solver
 */
#include "osqp++.h"

/*******************************************************************************
 * STL
 */
#include "vector"
#include "mutex"
#include "functional"

/*******************************************************************************
 * MVP
 */
#include "mimo_pid.h"

namespace ctrl {
/** @brief MvpControl Class
 *
 */
    class MvpControl {
    private:

        //! @brief Control allocation matrix
        Eigen::MatrixXd m_control_allocation_matrix;

        //! @brief MIMO PID Controller
        MimoPID::Ptr m_pid;

        //! @brief System State
        Eigen::VectorXd m_system_state;

        //! @brief Desired State
        Eigen::VectorXd m_desired_state;

        //! @brief Error State
        Eigen::VectorXd m_error_state;

        //! @brief Controlled freedoms
        std::vector<int> m_controlled_freedoms;

        //! @brief Servo speeds for OSQP constraints
        Eigen::VectorXd m_servo_speed;

        //! @brief Upper limits for OSQP forces boundary conditions
        Eigen::VectorXd m_upper_limit;

        //! @brief Lower limits for OSQP forces boundary conditions
        Eigen::VectorXd m_lower_limit;

        //! @brief Upper angle limits vector
        Eigen::VectorXd m_upper_angle;

        //! @brief Lower angle limits vector
        Eigen::VectorXd m_lower_angle;

        //! @brief Vector representing thruster configuration
        Eigen::VectorXd m_thruster_vector;

        //! @brief TF prefix for thruster transformations
        std::string m_tf_prefix_thruster;

        //! @brief Controlled grequency
        double m_controller_frequency; 

        //! @brief Index of the current thruster
        int m_thruster_index; 

        //! @brief the angle between the thruster and the x-axis
        double beta;

        //! @brief Store the current angles for each servo
        std::vector<double> m_current_angles; //

        //! @brief Adjusted upper limit for thruster constraints
        std::vector<int> m_adjusted_upper_limit;

        //! @brief Adjusted lower limit for thruster constraints
        std::vector<int> m_adjusted_lower_limit;

        //! @brief Constants for solver
        static constexpr double kInfinity = std::numeric_limits<double>::infinity();

        /** @brief Calculates PID using #MimoPID
         *
         * Measures the error between desired and current state.
         * Computes PID and generates a control input.
         * Resulting control input is written to the u.
         *
         * @param u Control input.
         * @param dt Time difference in seconds
         * @return
         */
        bool f_calculate_pid(Eigen::VectorXd *u, double dt);

        /** @brief Optimize thrust for given control input
         *
         * @param t Optimized forces for each thruster. This is the return value.
         * @param u Control input
         * @return
         */
        bool f_optimize_thrust(Eigen::VectorXd *t, Eigen::VectorXd u);

        /** @brief Error function for #MvpControl::m_pid object
         *
         * This method computes the error of each degree of freedom in their own domain.
         *
         * @param desired
         * @param current
         * @return Error between desired and current state
         */
        Eigen::ArrayXd f_error_function(
            Eigen::ArrayXd desired, Eigen::ArrayXd current);

        //! @brief Mutex lock for protect allocation matrix during changes
        std::recursive_mutex m_allocation_matrix_lock;

        //! @brief Mutex lock for protect controlled freedoms during changes
        std::recursive_mutex m_controlled_freedoms_lock;

        //! @brief Mutex lock for protect desired state during changes
        std::recursive_mutex m_desired_state_lock;

        //! @brief Mutex lock to protect thruster states during changes
        std::mutex m_thruster_vector_lock; 

    public:
        /**ns="alpha_control" @brief Mvp Control default constructor
         *
         */
        MvpControl();

        /** @brief Trivial Setter for control allocation matrix
         *
         * @param matrix
         */
        void set_control_allocation_matrix(
            const decltype(m_control_allocation_matrix) &matrix);

        /** @brief Trivial getter for thruster id
         *
         * @return #MvpControl::m_control_allocation_matrix
         */
        auto get_control_allocation_matrix() ->
        decltype(m_control_allocation_matrix);

        /** @brief Trivial Setter for articulation state vector
         *
         * @param vector
         */
        void set_thruster_articulation_vector(
            const decltype(m_thruster_vector) &vector);

        /**
         * @brief Trivial Setter for controller frequency
         *
         * @param frequency The new controller frequency value to set
         */
        void set_controller_frequency(
            const decltype(m_controller_frequency) &frequency);

        /**
         * @brief Getter for controller frequency
         *
         * @return The current controller frequency value
         */
        decltype(m_controller_frequency) get_controller_frequency() const {
            return m_controller_frequency;
        }

        /**
         * @brief Trivial Setter for TF prefix
         *
         * @param prefix The new TF prefix value to set
         */
        void set_tf_prefix(const std::string &prefix) {
            m_tf_prefix_thruster = prefix;
        }

        /**
         * @brief Getter for TF prefix
         *
         * @return The current TF prefix value
         */
        std::string get_tf_prefix() const {
            return m_tf_prefix_thruster;
        }

        /** @brief Trivial getter for thruster articulation vector
         *
         * @return The thruster articulation vector
         */
        auto get_thruster_articulation_vector() ->
        decltype(m_thruster_vector);

        //! @brief Standard shared pointer type
        typedef std::shared_ptr<MvpControl> Ptr;

        /** @brief Trivial getter for pid controller
         *
         * @return #MvpControl::m_pid
         */
        auto get_pid() -> decltype(m_pid);

        /** @brief Trivial setter for pid controller
         *
         * @param pid
         */
        void set_pid(const decltype(m_pid) &pid);

        /** @brief Trivial getter for system state
         *
         * @return #MvpControl::m_system_state
         */
        auto get_system_state() -> decltype(m_system_state);

        /** @brief Trivial setter for system state
         *
         * @param system_state
         */
        void set_system_state(const decltype(m_system_state) &system_state);

        /** @brief Trivial getter for desired state
         *
         * @return #MvpControl::m_desired_state
         */
        auto get_desired_state() -> decltype(m_desired_state);

        /** @brief Trivial setter for desired state
         *
         * @param desired_state
         */
        void set_desired_state(const decltype(m_desired_state) &desired_state);

        /** @brief Calculates needed forces from thrusters
         *
         * @param f     Force in body frame
         * @param dt    Time difference in seconds
         * @return      status of the operation.
         */
        bool calculate_needed_forces(Eigen::VectorXd *f, double dt);

        /** @brief Sets controlled freedoms
         *
         * @param f Degrees of freedom. See #MvpControlRos::m_controlled_freedoms
         */
        void set_controlled_freedoms(decltype(m_controlled_freedoms) f);

        /** @brief Get state error
         *
         * @return #MvpControlROS::m_error_state
         */
        auto get_state_error() -> decltype(m_error_state);

        /** @brief Update control allocation matrix
         *
         * Thread safe operation for updating control allocation matrix.
         * @param m Updated control allocation matrix
         */
        void update_control_allocation_matrix(
            const decltype(m_control_allocation_matrix) &m);

        /** @brief Update degrees of freedom
         *
         * Thread safe operation for updating controlled degrees of freedom
         * @param freedoms List of freedoms
         */
        void update_freedoms(std::vector<int> freedoms);

        /** @brief Update desired state
         *
         * Thread safe operation for updating desired state
         * @param desired_state
         */
        void
        update_desired_state(const decltype(m_desired_state) &desired_state);

        /** @brief Set the current angle.
         *
         * Updates the current angle of the specified thruster.
         * @param m_thruster_index Pointer to the index of the thruster.
         * @param angle The new current angle to set.
         */
        void set_current_angle(const int* m_thruster_index, double angle);

        /** @brief Get the current angle.
         *
         * Retrieves the current angle of the specified thruster.
         * @param m_thruster_index Pointer to the index of the thruster.
         * @return The current angle.
         */
        double get_current_angle(const int* m_thruster_index) const;

        /** @brief Set the lower limit for OSQP boundary conditions
        *
        * Updates the lower limit of the system for OSQP boundary conditions.
        * @param lower_limit The new lower limit to set.
        */
        void set_lower_limit(const decltype(m_lower_limit) &lower_limit);

        /** @brief Set the upper limit for OSQP boundary conditions
        *
        * Updates the upper limit of the system for OSQP boundary conditions.
        * @param upper_limit The new upper limit to set.
        */
        void set_upper_limit(const decltype(m_upper_limit) &upper_limit);

        /** @brief Set the lower angle limit
         *
         * Updates the lower angle limit of the system.
         * @param lower_angle The new lower angle limit to set.
         */
        void set_lower_angle(const decltype(m_lower_angle) &lower_angle);

        /** @brief Set the upper angle limit
         *
         * Updates the upper angle limit of the system.
         * @param upper_angle The new upper angle limit to set.
         */
        void set_upper_angle(const decltype(m_upper_angle) &upper_angle);

        /** @brief Set the servo speed
         *
         * Updates the servo speed of the system.
         * @param servo_speed The new servo speed to set.
         */
        void set_servo_speed(const decltype(m_servo_speed) &servo_speed);

    };

}