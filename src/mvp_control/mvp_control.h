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

        Eigen::VectorXd m_upper_limit;

        Eigen::VectorXd m_lower_limit;

        Eigen::VectorXd m_thruster_vector;

        //! @brief Controlled grequency
        int m_controller_frequency; 

        //! @brief Current angles for each joint or thruster
        std::unordered_map<std::string, double> m_current_angles;
        
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

        /** @brief Set the current angle
        *
        * Updates the current angle of the system.
        * @param angle The new current angle to set.
        */
        void set_current_angle(double angle);

        /** @brief Get the current angle
        *
        * Retrieves the current angle of the system.
        * @return The current angle.
        */
        double get_current_angle() const;
        
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


    };

}