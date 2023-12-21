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
 * STL
 */
#include "functional"
#include "thread"

/*******************************************************************************
 * ROS
 */
#include "ros/ros.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <urdf/model.h>

#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "dynamic_reconfigure/server.h"

#include "mvp_control/PIDConfig.h"

#include "mvp_msgs/PIDgains.h"
#include "mvp_msgs/ControlProcess.h"
#include "mvp_msgs/ControlModes.h"
#include "mvp_msgs/GetControlMode.h"
#include "mvp_msgs/GetControlModes.h"
#include "mvp_msgs/SetControlPoint.h"

/*******************************************************************************
 * MVP
 */

#include "mvp_control.h"
#include "thruster_ros.h"


/*******************************************************************************
 * Eigen
 */

#include "Eigen/Dense"

/*******************************************************************************
 * BOOST
 */
#include "boost/thread/recursive_mutex.hpp"


namespace ctrl {
/** @brief ROS wrapper for MvpControl
 *
 *  This package wraps mvp control class and utilizes its
 *  functionality.
 *
 *  @see MvpControl
 */
    class MvpControlROS {
    private:
        /*! @brief Generator Type enum class
         *
         */
        enum class GeneratorType {
            TF,
            USER,
            UNKNOWN
        };

        //! @brief Public node handler
        ros::NodeHandle m_nh;

        //! @brief Private node handler
        ros::NodeHandle m_pnh;

        //! @brief Thruster list
        std::vector<ThrusterROS::Ptr> m_thrusters;

        //! @brief Servos list
        std::vector<ThrusterROS::Ptr> m_servos;

        /**! @brief Control Allocation Matrix
         *
         *  Control allocation matrix is generated from individual
         *  configurations of the thrusters.
         */
        Eigen::MatrixXd m_control_allocation_matrix;

        Eigen::MatrixXd m_control_allocation_matrix_previous;

        /**! @brief Thruster Articulation
         *  Thrusters articulation flag
         *  passed to this vector equal to the 
         *  size of atriculation matrix columns
         */
        Eigen::VectorXd m_thruster_vector;

        //! @brief Control allocation matrix generator type
        GeneratorType m_generator_type;

        //! @brief Center of gravity link id
        std::string m_cg_link_id;

        //! @brief World link id
        std::string m_world_link_id;

        //! @brief Controller Frequency Param
        double m_controller_frequency;

        //! @brief Transform buffer for TF2
        tf2_ros::Buffer m_transform_buffer;

        //! @brief Transform listener for TF2
        tf2_ros::TransformListener m_transform_listener;

        //! @brief Transform prefix
        std::string m_tf_prefix;

        //! @brief Mvp Control object
        MvpControl::Ptr m_mvp_control;

        //! @brief Process values
        Eigen::VectorXd m_process_values;

        //! @brief Set point
        Eigen::VectorXd m_set_point;

        //! @brief Get control modes ros service server
        ros::ServiceServer m_get_control_modes_server;

        //! @brief Set control point ros service server
        ros::ServiceServer m_set_control_point_server;

        //! @brief Enable controller ros service server
        ros::ServiceServer m_enable_controller_server;

        //! @brief Disable controller ros service server
        ros::ServiceServer m_disable_controller_server;

        //! @brief Active mode getter ros service server
        ros::ServiceServer m_get_active_mode_server;

        //! @brief Trivial subscriber
        ros::Subscriber m_odometry_subscriber;

        //! @brief Process value publisher
        ros::Publisher m_process_value_publisher;

        //! @brief Set point subscriber
        ros::Subscriber m_set_point_subscriber;

        //! @brief Joint State subscriber
        ros::Subscriber m_joint_state_subscriber;

        sensor_msgs::JointState m_latest_joint_state;

        std::recursive_mutex m_joint_state_lock;

        //! @brief Joint state publisher
        ros::Publisher m_joint_state_publisher;

        //! @brief Publishes process error publisher
        ros::Publisher m_process_error_publisher;

        //! @brief Holder for latest odometry msg
        nav_msgs::Odometry m_odometry_msg;

        //! @brief Control modes stored as ros message
        mvp_msgs::ControlModes m_control_modes;

        //! @brief Desired state of the system, set point of the controller
        mvp_msgs::ControlProcess m_set_point_msg;

        //! @brief Current control mode
        std::string m_control_mode;

        //! @brief Protects odometry_msg during changes
        std::recursive_mutex m_odom_lock;

        /**
         * @brief Protects dynamic reconfigure server from dead locks
         * @todo In the future, boost might be replaced with std
         */
        boost::recursive_mutex m_config_lock;

        //! @brief Controller worker
        std::thread m_controller_worker;

        //! @brief A variable holds the controller status
        bool m_enabled;

        //! @brief Dynamic configure server for PID configuration
        std::shared_ptr<dynamic_reconfigure::Server<mvp_control::PIDConfig>>
            m_dynconf_pid_server;

        /** @brief Generates control allocation matrix from transform tree
         *
         *  This method is called if generator_type is 'tf'
         */
        void f_generate_control_allocation_from_tf();

        /** @brief [WIP]Generates control allocation matrix from transform tree
         *
         *  This method is for test
         */
        void f_generate_control_allocation_from_tf_2();

        /** @brief Generates control allocation matrix from user input
         *
         *  This method is called if generator_type is 'user'
         */
        void f_generate_control_allocation_from_user();

        /**!
         * @brief Generates control allocation matrix
         * This method reads the thruster configuration and generates the
         * initial matrix. It meant to be called once.
         */
        void f_generate_control_allocation_matrix();

        /**!
         * @brief Generates control allocation matrix
         * This sketch is none of anyone's business untill 
         * I clean the mess.
         */
        void f_generate_control_allocation_matrix_2();

        /**
         * @brief
         * @return
         */
        bool f_update_control_allocation_matrix();

        /** @brief Extract servo limits from URDF
         * 
         * This method extracts the upper and lower limits of the servo joints from the 
         * URDF model and updates the provided limit vectors.
         * 
         * @param model The URDF model to extract limits from.
         * @param upper_limit Vector to store the upper limits.
         * @param lower_limit Vector to store the lower limits.
         */
        void f_extractServoLimits(const urdf::Model& model, Eigen::VectorXd& upper_limit, Eigen::VectorXd& lower_limit);

        /** @brief Generate thrusters
         *
         */
        void f_generate_thrusters();

        /** @brief Reads ros param server and generates control modes
         *
         */
        void f_read_control_modes();

        /** @brief Compute process values
         *
         */
        bool f_compute_process_values();

        /** @brief Control Loop
         *
         *
         */
        void f_control_loop();

        /** @brief Retrieves the current position of a specified joint.
         *
         * This method looks up the current position (angle in radians) of a joint given its name. 
         * It is typically used to obtain real-time joint angles from the robot's state, which may 
         * be stored internally or received from a ROS topic. This functionality is crucial for 
         * control tasks that require knowledge of the robot's current configuration.
         *
         * @param joint_name The name of the joint for which the current position is requested.
         * @return The current position of the joint in radians. Throws an exception if the joint name is not found.
         */
        double f_get_current_joint_position(const std::string& joint_name);

        /** @brief Convert prq to world_frame angular rate:
         *  Eq.(2.12), Eq.(2.14) from Thor I. Fossen, Guidance and Control of Ocean Vehicles, Page 10
         *
         */
        Eigen::MatrixXd f_angular_velocity_transform(const Eigen::VectorXd& orientation);

        /** @brief Amends changes to Dynamic reconfigure server
         *
         * After reading the static configuration file, applies configuration to
         * dynamic reconfigure server.
         */
        void f_amend_dynconf();

        /** @brief Amends the control mode
         *
         * Changes the active control mode of the controller. Checks if request mode
         * is exist or not. Returns false if the operation requested is invalid.
         *
         * @param mode
         * @return
         */
        bool f_amend_control_mode(std::string mode);

        /** @brief Amends controllers set point
         *
         * Returns false if set point mode is invalid.
         * See #MvpControlROS::f_amend_control_mode
         *
         * @param set_point
         * @return
         */
        bool f_amend_set_point(const mvp_msgs::ControlProcess &set_point);

        /** @brief Trivial subscriber
         *
         * @param msg
         */
        void f_cb_msg_odometry(const nav_msgs::Odometry::ConstPtr &msg);

        /** @brief Trivial set point callback
         *
         * @param msg
         */
        void f_cb_srv_set_point(
            const mvp_msgs::ControlProcess::ConstPtr &msg);

        /** @brief Dynamic reconfigure server callback
         *
         * @param config
         * @param level
         */
        void f_cb_dynconf_pid(mvp_control::PIDConfig &config, uint32_t level);

        /** @brief Trivial ros service server callback for get control modes
         *
         * This service returns configured control modes to ros service client
         *
         * @param req
         * @param resp
         * @return Success of the operation
         */
        bool f_cb_srv_get_control_modes(
            mvp_msgs::GetControlModes::Request &req,
            mvp_msgs::GetControlModes::Response &resp);

        /** @brief Trivial ros service server callback for set control point
         *
         * @param req
         * @param resp
         * @return Success of the operation
         */
        bool f_cb_srv_set_control_point(
            mvp_msgs::SetControlPoint::Request req,
            mvp_msgs::SetControlPoint::Response resp);

        /**
         * @brief Enables the controller
         *
         * @param req Trivial empty request
         * @param resp Trivial empty response
         * @return true
         * @return false
         */
        bool f_cb_srv_enable(
            std_srvs::Empty::Request req,
            std_srvs::Empty::Response resp);

        /**
         * @brief disable the controller
         *
         * @param req Trivial empty request
         * @param resp Trivial empty response
         * @return true
         * @return false
         */
        bool f_cb_srv_disable(
            std_srvs::Empty::Request req,
            std_srvs::Empty::Response resp);

        /**
         * @brief Get active control mode
         * @param req Empty request
         * @param resp Response with current control mode
         * @return true Always returns true
         */
        bool f_cb_srv_get_active_mode(
            mvp_msgs::GetControlMode::Request &req,
            mvp_msgs::GetControlMode::Response &resp);

    public:

        /** @brief Default constructor
         *
         */
        MvpControlROS();

        /** @brief Initializer for Mvp Control ROS
         *
         * This function initializes control allocation matrix and
         *
         */
        void initialize();

        void f_cb_msg_joint_state(const sensor_msgs::JointState::ConstPtr &msg);

        //! @brief Generic typedef for shared pointer
        typedef std::shared_ptr<MvpControlROS> Ptr;

        void f_log_latest_joint_state();

        /**
         * @brief Publishes joint states dynamically.
         * @param joint_names Names of the joints.
         * @param positions Positions for each joint.
         * @param velocities (Optional) Velocities for each joint.
         * @param efforts (Optional) Efforts for each joint.
         */
        void f_publish_joint_states(const std::vector<std::string>& joint_names, const std::vector<double>& positions, const std::vector<double>& velocities = {}, const std::vector<double>& efforts = {});

    };

}