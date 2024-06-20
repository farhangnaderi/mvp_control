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

#include "mvp_control_ros.h"
#include "exception.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "mvp_control/dictionary.h"
#include "boost/regex.hpp"

using namespace ctrl;

MvpControlROS::MvpControlROS()
        : m_nh(),
        m_pnh("~"),
        m_transform_listener(m_transform_buffer),
        m_generator_type(MvpControlROS::GeneratorType::UNKNOWN)
{

    m_process_values = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    m_set_point = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    /**
     * Read basic configuration. Configuration regarding to thruster allocation
     * will be read later.
     */

    // Read configuration: enabled
    m_pnh.param<bool>(CONF_ENABLED, m_enabled, false);

    // Read configuration: tf prefix
    std::string tf_prefix;
    m_pnh.param<std::string>(CONF_TF_PREFIX, tf_prefix, CONF_TF_PREFIX_DEFAULT);
    m_tf_prefix = tf_prefix.empty() ? CONF_TF_PREFIX_DEFAULT : tf_prefix + "/";

    // Read configuration: center of gravity link
    std::string cg_link_id;
    m_pnh.param<std::string>(CONF_CG_LINK, cg_link_id, CONF_CG_LINK_DEFAULT);
    m_cg_link_id = m_tf_prefix + cg_link_id;

    // Read configuration: world link
    m_pnh.param<std::string>(
            CONF_WORLD_LINK,
            m_world_link_id,
            CONF_WORLD_LINK_DEFAULT
    );

    // Read configuration: odometry topic id
    std::string odometry_topic;
    m_pnh.param<std::string>(
            CONF_ODOMETRY_SOURCE,
            odometry_topic,
            CONF_ODOMETRY_SOURCE_DEFAULT
    );

    // Read configuration: joints topic id
    std::string joint_states_topic;
    m_pnh.param<std::string>(
            CONF_SERVO_JOINT_TOPIC,
            joint_states_topic, ""
    );

    if (joint_states_topic.empty()) {
        ROS_ERROR("The joint_states_topic parameter is not set!");
    }

    m_pnh.param<double>(
        CONF_CONTROLLER_FREQUENCY,
        m_controller_frequency,
        10.0
    );

    /**
     * Initialize Subscribers
     */
    m_odometry_subscriber = m_nh.subscribe(
            odometry_topic,
            100,
            &MvpControlROS::f_cb_msg_odometry,
            this
    );

    m_joint_state_subscriber = m_nh.subscribe<sensor_msgs::JointState>(
            joint_states_topic,
            100,
            &MvpControlROS::f_cb_msg_joint_state,
            this
    );

    m_set_point_subscriber = m_nh.subscribe(
        TOPIC_CONTROL_PROCESS_SET_POINT,
        100,
        &MvpControlROS::f_cb_srv_set_point,
        this
    );

    /**
     * Initialize publishers
     */
    m_process_value_publisher = m_nh.advertise<mvp_msgs::ControlProcess>(
        TOPIC_CONTROL_PROCESS_VALUE,
        100
    );

    m_process_error_publisher = m_nh.advertise<mvp_msgs::ControlProcess>(
        TOPIC_CONTROL_PROCESS_ERROR,
        100
    );

    /**
     * Initialize services
     */
    m_get_control_modes_server = m_nh.advertiseService
        <mvp_msgs::GetControlModes::Request,
        mvp_msgs::GetControlModes::Response>
    (
        SERVICE_GET_CONTROL_MODES,
        std::bind(
            &MvpControlROS::f_cb_srv_get_control_modes,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    m_set_control_point_server = m_nh.advertiseService
        <mvp_msgs::SetControlPoint::Request,
        mvp_msgs::SetControlPoint::Response>
    (
        SERVICE_SET_CONTROL_POINT,
        std::bind(
            &MvpControlROS::f_cb_srv_set_control_point,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    m_enable_controller_server = m_nh.advertiseService
        <std_srvs::Empty::Request,
        std_srvs::Empty::Response>
    (
        SERVICE_CONTROL_ENABLE,
        std::bind(
            &MvpControlROS::f_cb_srv_enable,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    m_disable_controller_server = m_nh.advertiseService
        <std_srvs::Empty::Request,
        std_srvs::Empty::Response>
    (
        SERVICE_CONTROL_DISABLE,
        std::bind(
            &MvpControlROS::f_cb_srv_disable,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    m_get_controller_state_server = m_nh.advertiseService
        <std_srvs::Trigger::Request,
        std_srvs::Trigger::Response>
    (
        SERVICE_GET_CONTROLLER_STATE,
        std::bind(
            &MvpControlROS::f_cb_srv_get_controller_state,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    m_get_active_mode_server = m_nh.advertiseService
        <mvp_msgs::GetControlMode::Request,
        mvp_msgs::GetControlMode::Response>
    (
        SERVICE_GET_ACTIVE_MODE,
        std::bind(
            &MvpControlROS::f_cb_srv_get_active_mode,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    /**
     * Initialize dynamic reconfigure server
     */
    m_dynconf_pid_server.reset(
        new dynamic_reconfigure::Server<mvp_control::PIDConfig>(
            m_config_lock)
        );

    /**
     * Initialize the actual controller
     */
    m_mvp_control.reset(new MvpControl());

}

void MvpControlROS::f_generate_control_allocation_matrix() {

    // Read generator type (e.g., TF or User-defined)
    std::string generator_type;
    m_pnh.param<std::string>(
            CONF_GENERATOR_TYPE,
            generator_type,
            CONF_GENERATOR_TYPE_OPT_TF
    );

    // Parse the control allocation generator type and save it as enum type
    if(generator_type == CONF_GENERATOR_TYPE_OPT_TF) {
        m_generator_type = GeneratorType::TF;
    } else if (generator_type == CONF_GENERATOR_TYPE_OPT_USER) {
        m_generator_type = GeneratorType::USER;
    } else {
        m_generator_type = GeneratorType::UNKNOWN;
    }

    // Generate the control allocation matrix based on the specified generator type
    if(m_generator_type == GeneratorType::USER) {
        f_generate_control_allocation_from_user();
    } else if (m_generator_type == GeneratorType::TF) {
        f_generate_control_allocation_from_tf();
    } else {
        throw control_ros_exception(
            "control allocation generation method unspecified"
        );
    }
       
    // Conduct some checks to ensure everything is ready for initialization
    if(m_thrusters.empty()) {
        throw control_ros_exception("no thruster specified");
    } else {
        for(size_t i = 0; i < m_thrusters.size(); i++) {
            std::string id = m_thrusters[i]->get_id();
            int isArticulated = m_thrusters[i]->get_is_articulated();
            std::vector<std::string> servoJoints = m_thrusters[i]->get_servo_joints();
            std::string link = m_thrusters[i]->get_link_id();
            
            // Log details of each thruster, including whether it's articulated and its associated joints
            ROS_INFO_STREAM("Thruster " << i << ": ID=" << id << ", Is Articulated=" << (isArticulated > 0 ? "Yes" : "No") << ", Link=" << link);
            
            if(isArticulated > 0) {
                std::string jointsStr;
                for(const auto& joint : servoJoints) {
                    jointsStr += joint + " ";
                }
                ROS_INFO_STREAM("  Joint: " << jointsStr);
            }
        }
    }

    // Control allocation matrix is generated based on each thruster. Each
    // thruster must have equal number of elements in their contribution matrix.
    // Code below checks the validity of the contribution vectors for each
    // thruster.
    for(int i = 0 ; i < m_thrusters.size() - 1 ; i++ ) {
        if (m_thrusters[i]->get_contribution_vector().size() !=
            m_thrusters[i + 1]->get_contribution_vector().size()) {
            throw control_ros_exception(
                "contribution vector sizes doesn't match"
            );
        }
    }
    Eigen::VectorXd m_thruster_vector = Eigen::VectorXd::Zero(m_thrusters.size());

    for (int i = 0; i < m_thrusters.size(); i++) {
        m_thruster_vector[i] = m_thrusters[i]->get_is_articulated();
    }

    m_mvp_control->set_thruster_articulation_vector(m_thruster_vector);
    m_mvp_control->set_controller_frequency(m_controller_frequency);
    m_mvp_control-> set_tf_prefix(m_tf_prefix);
    
    // Initialize the control allocation matrix with zeros.
    // M -> number of controllable DOFs, N -> number of thrusters
    m_control_allocation_matrix = Eigen::MatrixXd::Zero(
        CONTROLLABLE_DOF_LENGTH, (int) m_thrusters.size()
    );

    // Until this point, all the allocation matrix related issued must be
    // solved or exceptions thrown.

    // Register each DOF per actuator to the control allocation matrix.
    // Only DOFs X, Y, and Z are left unregistered. They are computed
    // online after each iteration.
    for (int i = 0; i < m_thrusters.size(); i++) {
        for(const auto& j :
            {DOF::ROLL, DOF::PITCH, DOF::YAW,
             DOF::SURGE, DOF::SWAY, DOF::HEAVE,
             DOF::ROLL_RATE, DOF::PITCH_RATE, DOF::YAW_RATE
             })
        {
            m_control_allocation_matrix(j, i) =
                m_thrusters[i]->get_contribution_vector()(j);
        }
    }

    // Set the final control allocation matrix for the controller object
    m_mvp_control->set_control_allocation_matrix(m_control_allocation_matrix);

}

void MvpControlROS::f_generate_thrusters() {

    if (!m_pnh.hasParam(CONF_THRUSTER_IDS)) {
        throw control_ros_exception("thruster_ids empty");
    }

    std::vector<std::string> thruster_id_list;
    m_pnh.getParam(CONF_THRUSTER_IDS, thruster_id_list);

    std::map<std::string, std::string> thruster_servo_joints;
    m_pnh.getParam(CONF_THRUSTER_SERVO_JOINTS, thruster_servo_joints);

    // First Stage: Initialization with Articulated Check
    for (const auto& id : thruster_id_list) {
        
        // Check if the current thruster is articulated
        auto it = thruster_servo_joints.find(id);
        int isArticulated = (it != thruster_servo_joints.end()) ? 1 : 0;
       // Create a ThrusterROS object for the non-articulated or the first articulated joint
        ThrusterROS::Ptr t(new ThrusterROS());
        t->set_id(id);
        t->set_is_articulated(isArticulated);

        if (isArticulated == 0) {
            // Thruster is not articulated
            ROS_INFO_STREAM("Thruster " << id << " is not articulated.");
            // Add the non-articulated thruster to the list
            m_thrusters.emplace_back(t);
        } else {
            /* 
            Thruster is articulated, create two ThrusterROS objects: 
            */
            ThrusterROS::Ptr articulated_tx(new ThrusterROS());
            articulated_tx->set_id(id);
            articulated_tx->set_is_articulated(1);
            articulated_tx->set_servo_joints({it->second}); // Set the first servo joint
            
            // Log the first servo joint
            ROS_INFO_STREAM("Thruster " << id << " is articulated with servo joint: " << it->second);
            
            m_thrusters.emplace_back(articulated_tx);

            ThrusterROS::Ptr articulated_ty(new ThrusterROS());
            articulated_ty->set_id(id);
            articulated_ty->set_is_articulated(2);
            articulated_ty->set_servo_joints({it->second}); // Set the second servo joint
            
            // Log the second servo joint
            ROS_INFO_STREAM("Thruster " << id << " is articulated with servo joint: " << it->second);
            
            m_thrusters.emplace_back(articulated_ty);
        }
    }

    // Second Stage: Detailed Configuration for Each Thruster
    for (const auto& t : m_thrusters) {
        std::string thrust_command_topic_id, thrust_force_topic_id;
        std::string servo_joint_desired_topic_id,servo_command_topic_id;
        std::vector<double> poly;
        double force_max, force_min;
        double angle_max, angle_min, omega;

        // Thrust command topic configuration
        m_pnh.param<std::string>(
            std::string(CONF_THRUST_COMMAND_TOPICS) + "/" + t->get_id(), 
            thrust_command_topic_id, 
            "control/thruster/" + t->get_id() + "/command");
        t->set_thrust_command_topic_id(thrust_command_topic_id);

        // Thrust force topic configuration
        m_pnh.param<std::string>(
            std::string(CONF_THRUSTER_FORCE_TOPICS) + "/" + t->get_id(), 
            thrust_force_topic_id, 
            "control/thruster/" + t->get_id() + "/force");
        t->set_thrust_force_topic_id(thrust_force_topic_id);

        // Joint state topic configuration
        m_pnh.param<std::string>(
            "/" + m_tf_prefix + std::string(CONF_SERVO_JOINT_SETPOINT_TOPIC), 
            servo_joint_desired_topic_id, 
            "/" + m_tf_prefix + "control/servos/desired_joint_states");
        t->set_joint_state_desired_topic_id(servo_joint_desired_topic_id);
        
        // Servo command topic configuration
        m_pnh.param<std::string>(std::string(CONF_SERVO_COMMAND_TOPICS) + "/" + t->get_id(), 
                                servo_command_topic_id, 
                                "");
        t->set_servo_command_topic_id(servo_command_topic_id);

        // Polynomial coefficients configuration for thrusters
        m_pnh.param<std::vector<double>>(
            std::string(CONF_THRUSTER_POLY) + "/" + t->get_id(), 
            poly, std::vector<double>());
        t->get_poly_solver()->set_coeff(poly);

        // Read servo coefficients from the configuration file
        std::vector<double> servo_poly;
        m_pnh.param<std::vector<double>>(
            std::string(CONF_SERVO_POLY) + "/" + t->get_id(), 
            servo_poly, std::vector<double>());
        t->set_servo_coeff(servo_poly);

        // Servo speeds in rad/s
        if (!m_pnh.getParam(std::string(CONF_THRUSTER_SERVO_SPEEDS) + "/" + t->get_id(), omega)) {
            ROS_WARN("'%s' not set. Assuming as non-articulated and setting to zero.", 
                    (std::string(CONF_THRUSTER_SERVO_SPEEDS) + ":" + t->get_id()).c_str());
        } else {
            t->m_omega = omega;
        }

        // Force limits configuration
        m_pnh.param<double>(
            std::string(CONF_THRUSTER_LIMITS) + "/" + t->get_id() + "/" + CONF_THRUSTER_MAX, 
            force_max, 
            10.0);
        t->m_force_max = force_max;

        m_pnh.param<double>(
            std::string(CONF_THRUSTER_LIMITS) + "/" + t->get_id() + "/" + CONF_THRUSTER_MIN, 
            force_min, 
            -10.0);
        t->m_force_min = force_min;

        /*
        Angle limits configuration
        For safety the default angle values are passed zero 
        in case no input available in config file.
        */

        if (!m_pnh.getParam(std::string(CONF_SERVO_LIMITS) + "/" + t->get_id() + "/" + CONF_SERVO_MAX, angle_max)) {
            ROS_WARN(" '%s' not set. Assuming as non-articulated and setting to zero.",
                    (std::string(CONF_SERVO_LIMITS) + ":" + t->get_id() + ":" + CONF_SERVO_MAX).c_str());
        } else {
            t->m_angle_max = angle_max;
        }

        if (!m_pnh.getParam(std::string(CONF_SERVO_LIMITS) + "/" + t->get_id() + "/" + CONF_SERVO_MIN, angle_min)) {
            ROS_WARN(" '%s' not set. Assuming as non-articulated and setting to zero.",
                    (std::string(CONF_SERVO_LIMITS) + ":" + t->get_id() + ":" + CONF_SERVO_MIN).c_str());
        } else {
            t->m_angle_min = angle_min;
        }

    }
}

void MvpControlROS::initialize() {

    // Read configured control modes from the ROS parameter server
    f_read_control_modes();

    // Generate thrusters with the given configuration
     f_generate_thrusters();

    // Generate control allocation matrix with defined method
    f_generate_control_allocation_matrix();

    // Initialize thruster objects.
    std::for_each(m_thrusters.begin(),m_thrusters.end(),
        [](const ThrusterROS::Ptr& t){
            t->initialize();
        }
    );

    m_mvp_control->set_desired_state(m_set_point);

    m_mvp_control->set_system_state(m_process_values);

    m_controller_worker = std::thread([this] { f_control_loop(); });

    m_controller_worker.detach();

    m_dynconf_pid_server->setCallback(
        std::bind(
            &MvpControlROS::f_cb_dynconf_pid,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

}

void MvpControlROS::f_generate_control_allocation_from_user() {
    for(const auto& t : m_thrusters) {

        Eigen::VectorXd contribution_vector;

        std::vector<double> v;

        m_pnh.param<decltype(v)>(
            std::string() + CONF_CONTROL_ALLOCATION_MATRIX + "/" + t->get_id(),
            v,
            decltype(v)()
        );

        contribution_vector =
            Eigen::Map<Eigen::VectorXd>(&v[0], (int) v.size());

        t->set_contribution_vector(contribution_vector);
    }
}

void MvpControlROS::f_generate_control_allocation_from_tf() {

    for(const auto& t : m_thrusters) {
        std::string link_id;
        m_pnh.param<std::string>(
            std::string() + CONF_CONTROL_TF + "/" + t->get_id(),
            link_id,
            t->get_id() + "_thruster_link"
        );

        t->set_link_id(m_tf_prefix + link_id);
    }
    
    // For each thruster look up transformation
    for(const auto& t : m_thrusters) {

        Eigen::Isometry3d eigen_tf;
        try {
            // 
            auto tf_cg_thruster = m_transform_buffer.lookupTransform(
                m_cg_link_id,
                t->get_link_id(),
                ros::Time::now(),
                ros::Duration(10.0)
            );

            eigen_tf = tf2::transformToEigen(tf_cg_thruster);
        } catch(tf2::TransformException &e) {
            ROS_WARN_STREAM_THROTTLE(10, 
            std::string("Can't compute thruster tf between cg-thruster: ") + e.what());
            return;
        }

        Eigen::VectorXd contribution_vector(CONTROLLABLE_DOF_LENGTH);

        auto trans_xyz = eigen_tf.translation();

        //! Eq.(2.12), Eq.(2.14) from Thor I. Fossen, Guidance and Control of Ocean Vehicles, Page 10
        Eigen::Matrix3d ang_vel_tranform = Eigen::Matrix3d::Identity();
        try {
            // Transform center of gravity to world
            auto tf_torque = m_transform_buffer.lookupTransform(
                m_world_link_id,
                m_cg_link_id,
                ros::Time::now(),
                ros::Duration(10.0)
            );

            tf2::Quaternion quat;
            quat.setW(tf_torque.transform.rotation.w);
            quat.setX(tf_torque.transform.rotation.x);
            quat.setY(tf_torque.transform.rotation.y);
            quat.setZ(tf_torque.transform.rotation.z);

            Eigen::VectorXd process_values = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
            tf2::Matrix3x3(quat).getRPY(
                process_values(DOF::ROLL),
                process_values(DOF::PITCH),
                process_values(DOF::YAW)
            );

            ang_vel_tranform = f_angular_velocity_transform(process_values);
        } catch(tf2::TransformException &e) {
            ROS_WARN_STREAM_THROTTLE(10, std::string("Can't compute thruster tf between world-cg: ") + e.what());
            return;
        }

        double Fx, Fy, Fz; 

        Eigen::Vector3d transformedVector;

        switch (t->get_is_articulated()) {
            case 0: //Non-articulated thruster
                transformedVector = eigen_tf.rotation() * Eigen::Vector3d::UnitX();
                Fx = transformedVector.x(); 
                Fy = transformedVector.y();
                Fz = transformedVector.z();
                break;
            case 1: //Decoupled articulated thruster along X in trhuster frame
                transformedVector = eigen_tf.rotation() * Eigen::Vector3d::UnitX();
                Fx = transformedVector.x(); 
                Fy = transformedVector.y();
                Fz = transformedVector.z();
                break;
            case 2: //Decoupled articulated thruster along Y in trhuster frame
                transformedVector = eigen_tf.rotation() * Eigen::Vector3d::UnitY();
                Fx = transformedVector.x(); 
                Fy = transformedVector.y();
                Fz = transformedVector.z();
                break;
            default:
                ROS_WARN_STREAM("Invalid articulation index value for thruster " << t->get_id());
                break;
        }

        auto torque_pqr = trans_xyz.cross(Eigen::Vector3d{Fx, Fy, Fz});
        auto torque_rpy = ang_vel_tranform * torque_pqr;
        // body frame forces and torques
        contribution_vector(DOF::SURGE) = Fx;
        contribution_vector(DOF::SWAY) = Fy;
        contribution_vector(DOF::HEAVE) = Fz;
        contribution_vector(DOF::ROLL) = torque_rpy(0);
        contribution_vector(DOF::PITCH) = torque_rpy(1);
        contribution_vector(DOF::YAW) = torque_rpy(2);
        // body frame p,q,r
        contribution_vector(DOF::ROLL_RATE) = torque_pqr(0);
        contribution_vector(DOF::PITCH_RATE) = torque_pqr(1);
        contribution_vector(DOF::YAW_RATE) = torque_pqr(2);

        t->set_contribution_vector(contribution_vector);
    }
}

bool MvpControlROS::f_update_control_allocation_matrix() {

    // update control allocation based on actuators

    try {
        // Transform center of gravity to world
        auto cg_world = m_transform_buffer.lookupTransform(
            m_world_link_id,
            m_cg_link_id,
            ros::Time(0)
        );
        //only contiue the process if the tf is not too old

        if (abs(cg_world.header.stamp.toSec() - ros::Time::now().toSec()) < 10.0 || cg_world.header.stamp.toSec()==0.0) 
        { 
            auto tf_eigen = tf2::transformToEigen(cg_world);

            tf2::Quaternion quat;
            quat.setW(cg_world.transform.rotation.w);
            quat.setX(cg_world.transform.rotation.x);
            quat.setY(cg_world.transform.rotation.y);
            quat.setZ(cg_world.transform.rotation.z);

            Eigen::VectorXd orientation = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
            tf2::Matrix3x3(quat).getRPY(
                orientation(DOF::ROLL),
                orientation(DOF::PITCH),
                orientation(DOF::YAW)
            );

            Eigen::Matrix3d ang_vel_transform = Eigen::Matrix3d::Identity();

            // for each thruster compute contribution in earth frame
            for(int j = 0 ; j < m_control_allocation_matrix.cols() ; j++){

                Eigen::Isometry3d eigen_tf;
                try {
                    // Assuming m_thrusters[j] gives access to the j-th thruster object
                    auto tf_cg_thruster = m_transform_buffer.lookupTransform(
                        m_cg_link_id,
                        m_thrusters[j]->get_link_id(), 
                        ros::Time(0)
                    );

                    eigen_tf = tf2::transformToEigen(tf_cg_thruster);
                } catch(tf2::TransformException &e) {
                    ROS_WARN_STREAM_THROTTLE(10, std::string("Can't compute thruster tf: ") + e.what());
                    continue;
                }

                double Fx, Fy, Fz; 

                Eigen::Vector3d transformedVector;
                int isArticulated = m_thrusters[j]->get_is_articulated();

                switch (isArticulated) {
                    case 0: //Non-articulated thruster
                        transformedVector = eigen_tf.rotation() * Eigen::Vector3d::UnitX();
                        Fx = transformedVector.x(); 
                        Fy = transformedVector.y();
                        Fz = transformedVector.z();
                        break;
                    case 1: //Decoupled articulated thruster along X in trhuster frame
                        transformedVector = eigen_tf.rotation() * Eigen::Vector3d::UnitX();
                        Fx = transformedVector.x(); 
                        Fy = transformedVector.y();
                        Fz = transformedVector.z();
                        break;
                    case 2: //Decoupled articulated thruster along Y in trhuster frame
                        transformedVector = eigen_tf.rotation() * Eigen::Vector3d::UnitY();
                        Fx = transformedVector.x(); 
                        Fy = transformedVector.y();
                        Fz = transformedVector.z();
                        break;
                    default:
                        ROS_WARN_STREAM("Invalid articulation index value for thruster " << m_thrusters[j]->get_id());
                        break;
                }

                /*
                Forces and pqr have to be updated again in 
                case of rotation 
                */
                m_control_allocation_matrix(DOF::SURGE, j) = Fx;
                m_control_allocation_matrix(DOF::SWAY, j) = Fy;
                m_control_allocation_matrix(DOF::HEAVE, j) = Fz;

                Eigen::Vector3d uvw(Fx, Fy, Fz);

                Eigen::Vector3d xyz = tf_eigen.rotation() * uvw;

                m_control_allocation_matrix(DOF::X, j) = xyz(0);
                m_control_allocation_matrix(DOF::Y, j) = xyz(1);
                m_control_allocation_matrix(DOF::Z, j) = xyz(2);

                auto trans_xyz = eigen_tf.translation();
                auto torque_pqr = trans_xyz.cross(Eigen::Vector3d{Fx, Fy, Fz});

                // Convert prq to world_frame angular rate:
                //  Eq.(2.12), Eq.(2.14) from Thor I. Fossen, Guidance and Control of Ocean Vehicles, Page 10
                
                m_control_allocation_matrix(DOF::ROLL_RATE, j) = torque_pqr(0);
                m_control_allocation_matrix(DOF::PITCH_RATE, j) = torque_pqr(1),
                m_control_allocation_matrix(DOF::YAW_RATE, j) = torque_pqr(2);

                Eigen::Vector3d pqr(torque_pqr(0),torque_pqr(1),torque_pqr(2));               

                ang_vel_transform = f_angular_velocity_transform(orientation);

                auto rpy = ang_vel_transform * pqr;
                m_control_allocation_matrix(DOF::ROLL, j) = rpy(0);
                m_control_allocation_matrix(DOF::PITCH, j) = rpy(1);
                m_control_allocation_matrix(DOF::YAW, j) = rpy(2);             
            }
        }
        else
        {
            ROS_WARN( "%s to %s TF too old!", m_world_link_id.c_str(), m_cg_link_id.c_str() );
            return false;
        }

    } catch(tf2::TransformException& e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string("Can't update control allocation matrix ") + e.what());
        return false;
    }

    m_mvp_control->update_control_allocation_matrix(
        m_control_allocation_matrix
    );

    Eigen::VectorXd upper_limit(m_thrusters.size());
    Eigen::VectorXd lower_limit(m_thrusters.size());

    for(int i = 0; i < m_thrusters.size(); i++) {
        // Default values for upper and lower limits
        upper_limit[i] = m_thrusters[i]->m_force_max;
        lower_limit[i] = m_thrusters[i]->m_force_min;

    }

    m_mvp_control->set_lower_limit(lower_limit);

    m_mvp_control->set_upper_limit(upper_limit);

    // Define vectors for upper and lower angle limits
    Eigen::VectorXd angle_upper_limit(m_thrusters.size());
    Eigen::VectorXd angle_lower_limit(m_thrusters.size());

    for (int i = 0; i < m_thrusters.size(); i++) {
        // Check if m_angle_max is available
        if (m_thrusters[i]->m_angle_max != -1) {
            angle_upper_limit[i] = m_thrusters[i]->m_angle_max;
        } else {
            angle_upper_limit[i] = 0;
        }

        // Check if m_angle_min is available
        if (m_thrusters[i]->m_angle_min != -1) {
            angle_lower_limit[i] = m_thrusters[i]->m_angle_min;
        } else {
            angle_lower_limit[i] = 0;
        }
    }

    m_mvp_control->set_lower_angle(angle_lower_limit);
    m_mvp_control->set_upper_angle(angle_upper_limit);

    // Define vector for servo speeds
    Eigen::VectorXd servo_speed(m_thrusters.size());

    for (int i = 0; i < m_thrusters.size(); i++) {
        // Check if m_omega is available
        if (m_thrusters[i]->m_omega != -1) {
            servo_speed[i] = m_thrusters[i]->m_omega;
        } else {
            servo_speed[i] = 0;
        }
    }

    m_mvp_control->set_servo_speed(servo_speed);

    return true;
}

bool MvpControlROS::f_compute_process_values() {

    f_update_control_allocation_matrix();

    try {
        // Transform center of gravity to world
        auto cg_world = m_transform_buffer.lookupTransform(
            m_world_link_id,
            m_cg_link_id,
            ros::Time(0)
        );

        if (abs(cg_world.header.stamp.toSec() - ros::Time::now().toSec()) < 10 || cg_world.header.stamp.toSec()==0.0) 
        {
            tf2::Quaternion quat;
            quat.setW(cg_world.transform.rotation.w);
            quat.setX(cg_world.transform.rotation.x);
            quat.setY(cg_world.transform.rotation.y);
            quat.setZ(cg_world.transform.rotation.z);

            tf2::Matrix3x3(quat).getRPY(
                m_process_values(DOF::ROLL),
                m_process_values(DOF::PITCH),
                m_process_values(DOF::YAW)
            );

            m_process_values(DOF::X) = cg_world.transform.translation.x;
            m_process_values(DOF::Y) = cg_world.transform.translation.y;
            m_process_values(DOF::Z) = cg_world.transform.translation.z;
        }
        else
        {
            ROS_WARN( "%s to %s TF too old!", m_world_link_id.c_str(), m_cg_link_id.c_str() );
            return false;
        }

    } catch(tf2::TransformException &e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string("Can't compute process values: ") + e.what());
        return false;
    }
        // Transform from odom to world
    try{
        // std::scoped_lock lock(m_odom_lock);

        auto cg_odom = m_transform_buffer.lookupTransform(
                m_cg_link_id,
                m_odometry_msg.child_frame_id,
                ros::Time(0)
        );
        if (abs(cg_odom.header.stamp.toSec() - ros::Time::now().toSec()) < 10 || cg_odom.header.stamp.toSec() ==0.0) 
        {

            auto cg_odom_eigen = tf2::transformToEigen(cg_odom);

            // angular velocity from odomteyr_child_frame to cd_link
            tf2::Quaternion quat;
            quat.setW(cg_odom.transform.rotation.w);
            quat.setX(cg_odom.transform.rotation.x);
            quat.setY(cg_odom.transform.rotation.y);
            quat.setZ(cg_odom.transform.rotation.z);

            Eigen::VectorXd orientation = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
            tf2::Matrix3x3(quat).getRPY(
                orientation(DOF::ROLL),
                orientation(DOF::PITCH),
                orientation(DOF::YAW)
            );

            // convert linear velocity from odomtery to cg_link
            Eigen::Vector3d uvw;
            uvw(0) = m_odometry_msg.twist.twist.linear.x;
            uvw(1) = m_odometry_msg.twist.twist.linear.y;
            uvw(2) = m_odometry_msg.twist.twist.linear.z;

            uvw = cg_odom_eigen.rotation()  * uvw;

            m_process_values(DOF::SURGE) = uvw(0);
            m_process_values(DOF::SWAY) = uvw(1);
            m_process_values(DOF::HEAVE) = uvw(2);

            // convert angular velocity from odom_child_link to cg_link
            Eigen::Matrix3d ang_vel_transform = f_angular_velocity_transform(orientation);

            Eigen::Vector3d angular_rate;
            angular_rate(0) = m_odometry_msg.twist.twist.angular.x;
            angular_rate(1) = m_odometry_msg.twist.twist.angular.y;
            angular_rate(2) = m_odometry_msg.twist.twist.angular.z;

            angular_rate = ang_vel_transform * angular_rate;

            m_process_values(DOF::ROLL_RATE) = angular_rate(0);
            m_process_values(DOF::PITCH_RATE) = angular_rate(1);
            m_process_values(DOF::YAW_RATE) = angular_rate(2);
        }
        else
        {
            //printf("time %lf, dt=%lf \r\n", cg_odom.header.stamp.toSec(), ros::Time::now().toSec());
            ROS_WARN( "%s to %s TF too old!", m_cg_link_id.c_str(), m_odometry_msg.child_frame_id.c_str() );
            return false;
        }

    } catch(tf2::TransformException &e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string("Can't compute process values!, check odometry!: ") + e.what());
        return false;
    }

    mvp_msgs::ControlProcess s;
    s.header.stamp = ros::Time::now();
    s.header.frame_id = m_world_link_id;
    s.control_mode = m_control_mode;
    s.position.x = m_process_values(DOF::X);
    s.position.y = m_process_values(DOF::Y);
    s.position.z = m_process_values(DOF::Z);
    s.orientation.x = m_process_values(DOF::ROLL);
    s.orientation.y = m_process_values(DOF::PITCH);
    s.orientation.z = m_process_values(DOF::YAW);
    s.velocity.x = m_process_values(DOF::SURGE);
    s.velocity.y = m_process_values(DOF::SWAY);
    s.velocity.z = m_process_values(DOF::HEAVE);
    // body frame angular velocity pqr
    s.angular_rate.x = m_process_values(DOF::ROLL_RATE);
    s.angular_rate.y = m_process_values(DOF::PITCH_RATE);
    s.angular_rate.z = m_process_values(DOF::YAW_RATE);

    m_mvp_control->set_system_state(m_process_values);

    m_process_value_publisher.publish(s);

    mvp_msgs::ControlProcess e;

    Eigen::VectorXd error_state = m_mvp_control->get_state_error();
    e.header.stamp = ros::Time::now();
    e.header.frame_id = m_world_link_id;
    e.control_mode = m_control_mode;
    e.position.x = error_state(DOF::X);
    e.position.y = error_state(DOF::Y);
    e.position.z = error_state(DOF::Z);
    e.orientation.x = error_state(DOF::ROLL);
    e.orientation.y = error_state(DOF::PITCH);
    e.orientation.z = error_state(DOF::YAW);
    e.velocity.x = error_state(DOF::SURGE);
    e.velocity.y = error_state(DOF::SWAY);
    e.velocity.z = error_state(DOF::HEAVE);
    e.angular_rate.x = error_state(DOF::ROLL_RATE);
    e.angular_rate.y = error_state(DOF::PITCH_RATE);
    e.angular_rate.z = error_state(DOF::YAW_RATE);

    m_process_error_publisher.publish(e);
    // printf("end dt = %lf\r\n", ros::Time::now().toSec() - pt);

    return true;
}

void apply_action(Action action, double& Kp_surge, double& Ki_surge, double& Kd_surge, 
                  double& Kp_z, double& Ki_z, double& Kd_z, 
                  double& Kp_roll, double& Ki_roll, double& Kd_roll, 
                  double& Kp_sway, double& Ki_sway, double& Kd_sway, 
                  double& Kp_pitch, double& Ki_pitch, double& Kd_pitch, 
                  double& Kp_yaw, double& Ki_yaw, double& Kd_yaw) {
    double delta_p = 0.1, delta_i = 0.01, delta_d = 0.01;

    switch (action) {
        case INCREASE_KP_SURGE: Kp_surge += delta_p; break;
        case DECREASE_KP_SURGE: Kp_surge -= delta_p; break;
        case INCREASE_KI_SURGE: Ki_surge += delta_i; break;
        case DECREASE_KI_SURGE: Ki_surge -= delta_i; break;
        case INCREASE_KD_SURGE: Kd_surge += delta_d; break;
        case DECREASE_KD_SURGE: Kd_surge -= delta_d; break;

        case INCREASE_KP_Z: Kp_z += delta_p; break;
        case DECREASE_KP_Z: Kp_z -= delta_p; break;
        case INCREASE_KI_Z: Ki_z += delta_i; break;
        case DECREASE_KI_Z: Ki_z -= delta_i; break;
        case INCREASE_KD_Z: Kd_z += delta_d; break;
        case DECREASE_KD_Z: Kd_z -= delta_d; break;

        case INCREASE_KP_ROLL: Kp_roll += delta_p; break;
        case DECREASE_KP_ROLL: Kp_roll -= delta_p; break;
        case INCREASE_KI_ROLL: Ki_roll += delta_i; break;
        case DECREASE_KI_ROLL: Ki_roll -= delta_i; break;
        case INCREASE_KD_ROLL: Kd_roll += delta_d; break;
        case DECREASE_KD_ROLL: Kd_roll -= delta_d; break;

        case INCREASE_KP_SWAY: Kp_sway += delta_p; break;
        case DECREASE_KP_SWAY: Kp_sway -= delta_p; break;
        case INCREASE_KI_SWAY: Ki_sway += delta_i; break;
        case DECREASE_KI_SWAY: Ki_sway -= delta_i; break;
        case INCREASE_KD_SWAY: Kd_sway += delta_d; break;
        case DECREASE_KD_SWAY: Kd_sway -= delta_d; break;

        case INCREASE_KP_PITCH: Kp_pitch += delta_p; break;
        case DECREASE_KP_PITCH: Kp_pitch -= delta_p; break;
        case INCREASE_KI_PITCH: Ki_pitch += delta_i; break;
        case DECREASE_KI_PITCH: Ki_pitch -= delta_i; break;
        case INCREASE_KD_PITCH: Kd_pitch += delta_d; break;
        case DECREASE_KD_PITCH: Kd_pitch -= delta_d; break;

        case INCREASE_KP_YAW: Kp_yaw += delta_p; break;
        case DECREASE_KP_YAW: Kp_yaw -= delta_p; break;
        case INCREASE_KI_YAW: Ki_yaw += delta_i; break;
        case DECREASE_KI_YAW: Ki_yaw -= delta_i; break;
        case INCREASE_KD_YAW: Kd_yaw += delta_d; break;
        case DECREASE_KD_YAW: Kd_yaw -= delta_d; break;
    }

    // Cap the PID gains to prevent them from going out of control
    double max_p = 50.0, max_i = 20.0, max_d = 20.0;
    double min_p = 2.0, min_i = 0.0, min_d = 0.0;

    Kp_surge = std::clamp(Kp_surge, min_p, max_p);
    Ki_surge = std::clamp(Ki_surge, min_i, max_i);
    Kd_surge = std::clamp(Kd_surge, min_d, max_d);

    Kp_z = std::clamp(Kp_z, min_p, max_p);
    Ki_z = std::clamp(Ki_z, min_i, max_i);
    Kd_z = std::clamp(Kd_z, min_d, max_d);

    Kp_roll = std::clamp(Kp_roll, min_p, max_p);
    Ki_roll = std::clamp(Ki_roll, min_i, max_i);
    Kd_roll = std::clamp(Kd_roll, min_d, max_d);

    Kp_sway = std::clamp(Kp_sway, min_p, max_p);
    Ki_sway = std::clamp(Ki_sway, min_i, max_i);
    Kd_sway = std::clamp(Kd_sway, min_d, max_d);

    Kp_pitch = std::clamp(Kp_pitch, min_p, max_p);
    Ki_pitch = std::clamp(Ki_pitch, min_i, max_i);
    Kd_pitch = std::clamp(Kd_pitch, min_d, max_d);

    Kp_yaw = std::clamp(Kp_yaw, min_p, max_p);
    Ki_yaw = std::clamp(Ki_yaw, min_i, max_i);
    Kd_yaw = std::clamp(Kd_yaw, min_d, max_d);
}

// void MvpControlROS::f_control_loop() {

//     double pt = ros::Time::now().toSec();

//     auto r = ros::Rate(m_controller_frequency);

//     while(ros::ok()) {

//         /**
//          * Thread may not be able to sleep properly. This may happen using
//          * simulated time.
//          */
//         if(!r.sleep()) {
//             continue;
//         }

//         /**
//          * Check if controller is enabled or not.
//          */
//         if(!m_enabled) {
//              for(int i = 0 ; i < m_thrusters.size() ; i++) {
//                 m_thrusters.at(i)->command(0);
//             }
//             continue;
//         }

//         /**
//          * Compute the state of the system. Continue on failure. This may
//          * happen when transform tree is not ready.
//          */
//         if(not f_compute_process_values()) {
//             continue;
//         }

//         Eigen::VectorXd needed_forces;

//         /**
//          * Get time difference to feed PID controller
//          */
//         double dt = ros::Time::now().toSec() - pt;

//         /**
//          * Calculate forces to be requested from thrusters. If operation fails,
//          * do not send commands to thrusters.
//          */

//         if(m_mvp_control->calculate_needed_forces(&needed_forces, dt)) {
//             for(int i = 0; i < m_thrusters.size(); ) {
//                 auto is_articulated = m_thrusters.at(i)->get_is_articulated();

//                 int index = i;

//                 if(is_articulated == 1) {
//                     if(i + 1 < m_thrusters.size()) {
//                         auto combined_force = sqrt(pow(needed_forces(i), 2) + pow(needed_forces(i + 1), 2));
//                         m_thrusters.at(i)->request_force(combined_force);

//                         std::string thruster_link_id = m_thrusters.at(i)->get_link_id();
//                         double current_angle = 0.0;  // This will hold the computed angle in the x-z plane
//                         std::string joint_name = m_tf_prefix + m_thrusters.at(i)->get_servo_joints().at(0);

//                         try {
//                             auto tf_cg_thruster = m_transform_buffer.lookupTransform(
//                                 m_cg_link_id, 
//                                 thruster_link_id,  
//                                 ros::Time(0),
//                                 ros::Duration(3.0)
//                             );

//                             // Extract the rotation part as a quaternion
//                             tf2::Quaternion quaternion;
//                             tf2::fromMsg(tf_cg_thruster.transform.rotation, quaternion);

//                             // Convert quaternion to a rotation matrix
//                             tf2::Matrix3x3 rotation_matrix(quaternion);

//                             // Calculate the angle in the x-z plane
//                             tf2::Vector3 thruster_x_direction = rotation_matrix.getColumn(0); // UnitX direction in thruster frame
//                             thruster_x_direction.setY(0);  // Project onto the x-z plane
//                             thruster_x_direction.normalize();

//                             tf2::Vector3 cg_x_direction(1, 0, 0);  // X direction in cg_link's x-z plane

//                             // Compute the angle in radians using dot product
//                             double angle = acos(cg_x_direction.dot(thruster_x_direction));

//                             /*
//                             Determine the sign of the angle using cross product
//                             since acos returns the angle between 0 and pi always.
//                             */
//                             tf2::Vector3 cross_product = cg_x_direction.cross(thruster_x_direction);
//                             if (cross_product.z() < 0) {
//                                 angle = -angle;
//                             }

//                             current_angle = angle;

//                             m_mvp_control->set_current_angle(&index, current_angle);

//                         } catch (tf2::TransformException &ex) {
//                             ROS_WARN("%s", ex.what());
//                             continue; // Skip this iteration if the transform is unavailable
//                         }

//                         // Calculate the angle to be requested
//                         double x = needed_forces(i);
//                         double y = needed_forces(i + 1);
//                         double calculated_angle = atan2(y, x); 

//                         // Calculate the new angle since it is needed in body frame within -pi to pi
//                         double new_angle = current_angle + calculated_angle;
                        
//                         m_thrusters.at(i)->request_joint_angles(joint_name, new_angle);

//                         i += 2;  // Move to the next pair of articulated thrusters
//                     } else {
//                         ROS_WARN_STREAM("Expected articulated partner for thruster " << i << " but none found. Skipping.");
//                         i++; // Just move to next to avoid infinite loop in case of error
//                     }
//                 } else {
//                     m_thrusters.at(i)->request_force(needed_forces(i));
//                     //not articulated so no rotation state
//                     m_mvp_control->set_current_angle(&index, 0);
//                     i++; // Move to the next thruster
//                 }

//             }
//         }

//         /**
//          * Record the time that loop ends. Later, it will feed the PID
//          * controller.
//          */
//         pt = ros::Time::now().toSec();
//     }
// }

void MvpControlROS::f_control_loop() {
    double pt = ros::Time::now().toSec();
    auto r = ros::Rate(m_controller_frequency);

    // Initialize PID gains
    double Kp_surge = 2.0, Ki_surge = 1.0, Kd_surge = 0.5;
    double Kp_z = 1.0, Ki_z = 0.25, Kd_z = 0.1;
    double Kp_roll = 1.0, Ki_roll = 0.25, Kd_roll = 0.1;
    double Kp_sway = 1.0, Ki_sway = 0.5, Kd_sway = 0.5;
    double Kp_pitch = 1.0, Ki_pitch = 0.2, Kd_pitch = 0.5;
    double Kp_yaw = 1.0, Ki_yaw = 0.15, Kd_yaw = 0.5;

    // Initialize state variables
    State current_state = {0.0}, next_state = {0.0};
    Eigen::VectorXd prev_errors = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd integral_errors = Eigen::VectorXd::Zero(CONTROLLABLE_DOF_LENGTH);

    while (ros::ok()) {
        if (!r.sleep()) {
            continue;
        }

        if (!m_enabled) {
            for (int i = 0; i < m_thrusters.size(); i++) {
                m_thrusters.at(i)->command(0);
            }
            continue;
        }

        if (!f_compute_process_values()) {
            continue;
        }

        Eigen::VectorXd needed_forces;
        double dt = ros::Time::now().toSec() - pt;

        ROS_INFO("Before calculate_needed_forces");
        if (m_mvp_control->calculate_needed_forces(&needed_forces, dt)) {
            ROS_INFO("After calculate_needed_forces");

            for (int i = 0; i < m_thrusters.size();) {
                auto is_articulated = m_thrusters.at(i)->get_is_articulated();

                int index = i;

                if (is_articulated == 1) {
                    if (i + 1 < m_thrusters.size()) {
                        auto combined_force = sqrt(pow(needed_forces(i), 2) + pow(needed_forces(i + 1), 2));
                        m_thrusters.at(i)->request_force(combined_force);

                        std::string thruster_link_id = m_thrusters.at(i)->get_link_id();
                        double current_angle = 0.0;
                        std::string joint_name = m_tf_prefix + m_thrusters.at(i)->get_servo_joints().at(0);

                        try {
                            auto tf_cg_thruster = m_transform_buffer.lookupTransform(
                                m_cg_link_id,
                                thruster_link_id,
                                ros::Time(0),
                                ros::Duration(3.0)
                            );

                            Eigen::Isometry3d eigen_tf = tf2::transformToEigen(tf_cg_thruster);

                            Eigen::Vector3d thruster_x_direction = eigen_tf.rotation() * Eigen::Vector3d::UnitX();
                            thruster_x_direction.y() = 0;
                            thruster_x_direction.normalize();

                            Eigen::Vector3d cg_x_direction(1, 0, 0);

                            double angle = acos(cg_x_direction.dot(thruster_x_direction));

                            Eigen::Vector3d cross_product = cg_x_direction.cross(thruster_x_direction);
                            if (cross_product.z() < 0) {
                                angle = -angle;
                            }

                            current_angle = angle;

                            m_mvp_control->set_current_angle(&index, current_angle);

                        } catch (tf2::TransformException &ex) {
                            ROS_WARN("%s", ex.what());
                            continue;
                        }

                        double x = needed_forces(i);
                        double y = needed_forces(i + 1);
                        double calculated_angle = atan2(y, x);
                        double new_angle = current_angle + calculated_angle;

                        m_thrusters.at(i)->request_joint_angles(joint_name, new_angle);

                        i += 2;
                    } else {
                        ROS_WARN_STREAM("Expected articulated partner for thruster " << i << " but none found. Skipping.");
                        i++;
                    }
                } else {
                    m_thrusters.at(i)->request_force(needed_forces(i));
                    m_mvp_control->set_current_angle(&index, 0);
                    i++;
                }
            }
        }

        // Q-learning integration
        Eigen::VectorXd errors = m_mvp_control->get_state_error();
        Eigen::VectorXd d_errors = (errors - prev_errors) / dt;
        integral_errors += errors * dt;

        // Update current state
        current_state = {
            errors(DOF::SURGE), errors(DOF::Z), errors(DOF::ROLL),
            errors(DOF::SWAY), errors(DOF::PITCH), errors(DOF::YAW),
            d_errors(DOF::SURGE), d_errors(DOF::Z), d_errors(DOF::ROLL),
            d_errors(DOF::SWAY), d_errors(DOF::PITCH), d_errors(DOF::YAW),
            integral_errors(DOF::SURGE), integral_errors(DOF::Z), integral_errors(DOF::ROLL),
            integral_errors(DOF::SWAY), integral_errors(DOF::PITCH), integral_errors(DOF::YAW)
        };

        ROS_INFO("Before choose_action");
        Action action = choose_action(current_state);
        ROS_INFO("After choose_action");

        // Apply action to PID gains
        apply_action(action, Kp_surge, Ki_surge, Kd_surge, Kp_z, Ki_z, Kd_z, Kp_roll, Ki_roll, Kd_roll, Kp_sway, Ki_sway, Kd_sway, Kp_pitch, Ki_pitch, Kd_pitch, Kp_yaw, Ki_yaw, Kd_yaw);

        // Log PID gains and errors
        ROS_INFO("PID Gains: Kp_surge: %f, Ki_surge: %f, Kd_surge: %f, Kp_z: %f, Ki_z: %f, Kd_z: %f, Kp_roll: %f, Ki_roll: %f, Kd_roll: %f, Kp_sway: %f, Ki_sway: %f, Kd_sway: %f, Kp_pitch: %f, Ki_pitch: %f, Kd_pitch: %f, Kp_yaw: %f, Ki_yaw: %f, Kd_yaw: %f",
                 Kp_surge, Ki_surge, Kd_surge, Kp_z, Ki_z, Kd_z, Kp_roll, Ki_roll, Kd_roll, Kp_sway, Ki_sway, Kd_sway, Kp_pitch, Ki_pitch, Kd_pitch, Kp_yaw, Ki_yaw, Kd_yaw);
        ROS_INFO("Errors: Surge: %f, Sway: %f, Z: %f, Roll: %f, Pitch: %f, Yaw: %f",
                 errors(DOF::SURGE), errors(DOF::SWAY), errors(DOF::Z),
                 errors(DOF::ROLL), errors(DOF::PITCH), errors(DOF::YAW));

        // Calculate reward based on the new state
        double reward = -errors.norm() - 0.01 * (fabs(Kp_surge) + fabs(Ki_surge) + fabs(Kd_surge) +
                                                 fabs(Kp_z) + fabs(Ki_z) + fabs(Kd_z) +
                                                 fabs(Kp_roll) + fabs(Ki_roll) + fabs(Kd_roll) +
                                                 fabs(Kp_sway) + fabs(Ki_sway) + fabs(Kd_sway) +
                                                 fabs(Kp_pitch) + fabs(Ki_pitch) + fabs(Kd_pitch) +
                                                 fabs(Kp_yaw) + fabs(Ki_yaw) + fabs(Kd_yaw));
        ROS_INFO("Reward: %f", reward);

        // Update next state
        next_state = current_state;

        ROS_INFO("Before update_q_table");
        update_q_table(current_state, action, reward, next_state);
        ROS_INFO("After update_q_table");

        // Debug Q-table
        if (q_table.find(current_state) != q_table.end()) {
            ROS_INFO("Q-table entry for current state found. Q-values: ");
            for (size_t i = 0; i < q_table[current_state].size(); ++i) {
               // ROS_INFO("Action %zu: %f", i, q_table[current_state][i]);
            }
        } else {
            ROS_WARN("Q-table entry for current state not found.");
        }

        if (q_table.find(next_state) != q_table.end()) {
            ROS_INFO("Q-table entry for next state found. Q-values: ");
            for (size_t i = 0; i < q_table[next_state].size(); ++i) {
                //ROS_INFO("Action %zu: %f", i, q_table[next_state][i]);
            }
        } else {
            ROS_WARN("Q-table entry for next state not found.");
        }

        prev_errors = errors;
        pt = ros::Time::now().toSec();
    }
}

void MvpControlROS::f_cb_msg_odometry(
        const nav_msgs::Odometry::ConstPtr &msg) {
    std::scoped_lock lock(m_odom_lock);
    m_odometry_msg = *msg;
}

void MvpControlROS::f_cb_msg_joint_state(
        const sensor_msgs::JointState::ConstPtr &msg) {
    std::scoped_lock lock(m_joint_state_lock);
    m_latest_joint_state = *msg;
}

void MvpControlROS::f_cb_srv_set_point(
        const mvp_msgs::ControlProcess::ConstPtr &msg) {
    f_amend_set_point(*msg);
}

void MvpControlROS::f_cb_dynconf_pid(
        mvp_control::PIDConfig &config, uint32_t level) {

    Eigen::VectorXd p(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd i(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd d(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd i_max(CONTROLLABLE_DOF_LENGTH);
    Eigen::VectorXd i_min(CONTROLLABLE_DOF_LENGTH);


    p <<
            config.x_p,
            config.y_p,
            config.z_p,
            config.roll_p,
            config.pitch_p,
            config.yaw_p,
            config.surge_p,
            config.sway_p,
            config.heave_p,
            config.roll_rate_p,
            config.pitch_rate_p,
            config.yaw_rate_p;

    i <<
            config.x_i,
            config.y_i,
            config.z_i,
            config.roll_i,
            config.pitch_i,
            config.yaw_i,
            config.surge_i,
            config.sway_i,
            config.heave_i,
            config.roll_rate_i,
            config.pitch_rate_i,
            config.yaw_rate_i;

    d <<
            config.x_d,
            config.y_d,
            config.z_d,
            config.roll_d,
            config.pitch_d,
            config.yaw_d,
            config.surge_d,
            config.sway_d,
            config.heave_d,
            config.roll_rate_d,
            config.pitch_rate_d,
            config.yaw_rate_d;

    i_max <<
            config.x_i_max,
            config.y_i_max,
            config.z_i_max,
            config.roll_i_max,
            config.pitch_i_max,
            config.yaw_i_max,
            config.surge_i_max,
            config.sway_i_max,
            config.heave_i_max,
            config.roll_rate_i_max,
            config.pitch_rate_i_max,
            config.yaw_rate_i_max;

    i_min <<
            config.x_i_min,
            config.y_i_min,
            config.z_i_min,
            config.roll_i_min,
            config.pitch_i_min,
            config.yaw_i_min,
            config.surge_i_min,
            config.sway_i_min,
            config.heave_i_min,
            config.roll_rate_i_min,
            config.pitch_rate_i_min,
            config.yaw_rate_i_min;

    m_mvp_control->get_pid()->set_kp(p);
    m_mvp_control->get_pid()->set_ki(i);
    m_mvp_control->get_pid()->set_kd(d);
    m_mvp_control->get_pid()->set_i_max(i_max);
    m_mvp_control->get_pid()->set_i_min(i_min);

}

void MvpControlROS::f_amend_dynconf() {

    std::scoped_lock lock(m_config_lock);

    auto pid = m_mvp_control->get_pid();

    mvp_control::PIDConfig conf;

    conf.x_p = pid->get_kp()(DOF::X);
    conf.y_p = pid->get_kp()(DOF::Y);
    conf.z_p = pid->get_kp()(DOF::Z);
    conf.roll_p = pid->get_kp()(DOF::ROLL);
    conf.pitch_p = pid->get_kp()(DOF::PITCH);
    conf.yaw_p = pid->get_kp()(DOF::YAW);
    conf.surge_p = pid->get_kp()(DOF::SURGE);
    conf.sway_p = pid->get_kp()(DOF::SWAY);
    conf.heave_p = pid->get_kp()(DOF::HEAVE);
    conf.roll_rate_p = pid->get_kp()(DOF::ROLL_RATE);
    conf.pitch_rate_p = pid->get_kp()(DOF::PITCH_RATE);
    conf.yaw_rate_p = pid->get_kp()(DOF::YAW_RATE);

    conf.x_i = pid->get_ki()(DOF::X);
    conf.y_i = pid->get_ki()(DOF::Y);
    conf.z_i = pid->get_ki()(DOF::Z);
    conf.roll_i = pid->get_ki()(DOF::ROLL);
    conf.pitch_i = pid->get_ki()(DOF::PITCH);
    conf.yaw_i = pid->get_ki()(DOF::YAW);
    conf.surge_i = pid->get_ki()(DOF::SURGE);
    conf.sway_i = pid->get_ki()(DOF::SWAY);
    conf.heave_i = pid->get_ki()(DOF::HEAVE);
    conf.roll_rate_i = pid->get_ki()(DOF::ROLL_RATE);
    conf.pitch_rate_i = pid->get_ki()(DOF::PITCH_RATE);
    conf.yaw_rate_i = pid->get_ki()(DOF::YAW_RATE);

    conf.x_d = pid->get_kd()(DOF::X);
    conf.y_d = pid->get_kd()(DOF::Y);
    conf.z_d = pid->get_kd()(DOF::Z);
    conf.roll_d = pid->get_kd()(DOF::ROLL);
    conf.pitch_d = pid->get_kd()(DOF::PITCH);
    conf.yaw_d = pid->get_kd()(DOF::YAW);
    conf.surge_d = pid->get_kd()(DOF::SURGE);
    conf.sway_d = pid->get_kd()(DOF::SWAY);
    conf.heave_d = pid->get_kd()(DOF::HEAVE);
    conf.roll_rate_d = pid->get_kd()(DOF::ROLL_RATE);
    conf.pitch_rate_d = pid->get_kd()(DOF::PITCH_RATE);
    conf.yaw_rate_d = pid->get_kd()(DOF::YAW_RATE);

    conf.x_i_max = pid->get_i_max()(DOF::X);
    conf.y_i_max = pid->get_i_max()(DOF::Y);
    conf.z_i_max = pid->get_i_max()(DOF::Z);
    conf.roll_i_max = pid->get_i_max()(DOF::ROLL);
    conf.pitch_i_max = pid->get_i_max()(DOF::PITCH);
    conf.yaw_i_max = pid->get_i_max()(DOF::YAW);
    conf.surge_i_max = pid->get_i_max()(DOF::SURGE);
    conf.sway_i_max = pid->get_i_max()(DOF::SWAY);
    conf.heave_i_max = pid->get_i_max()(DOF::HEAVE);
    conf.roll_rate_i_max = pid->get_i_max()(DOF::ROLL_RATE);
    conf.pitch_rate_i_max = pid->get_i_max()(DOF::PITCH_RATE);
    conf.yaw_rate_i_max = pid->get_i_max()(DOF::YAW_RATE);

    conf.x_i_min = pid->get_i_min()(DOF::X);
    conf.y_i_min = pid->get_i_min()(DOF::Y);
    conf.z_i_min = pid->get_i_min()(DOF::Z);
    conf.roll_i_min = pid->get_i_min()(DOF::ROLL);
    conf.pitch_i_min = pid->get_i_min()(DOF::PITCH);
    conf.yaw_i_min = pid->get_i_min()(DOF::YAW);
    conf.surge_i_min = pid->get_i_min()(DOF::SURGE);
    conf.sway_i_min = pid->get_i_min()(DOF::SWAY);
    conf.heave_i_min = pid->get_i_min()(DOF::HEAVE);
    conf.roll_rate_i_min = pid->get_i_min()(DOF::ROLL_RATE);
    conf.pitch_rate_i_min = pid->get_i_min()(DOF::PITCH_RATE);
    conf.yaw_rate_i_min = pid->get_i_min()(DOF::YAW_RATE);

    m_dynconf_pid_server->updateConfig(conf);

}

void MvpControlROS::f_read_control_modes() {
    std::vector<std::string> params;
    m_pnh.getParamNames(params);

    /**
     * Read all the modes with regex
     */
    std::set<std::string> modes;
    for (const auto &i: params) {
        boost::regex e{
            std::string() + "(?<=" + CONF_CONTROL_MODES + "/)(\\w+)"};
        boost::smatch w;
        if (boost::regex_search(i, w, e)) {
            modes.insert(w[0]);
        }
    }

    if(modes.empty()) {
        /**
         * There is no mode detected by the control mode parser.
         */
         throw control_ros_exception(
             "No control mode configuration have been found."
         );
    }

    /**
     * Read all the degrees of freedoms by a mode
     */
    std::map<std::string, std::set<int>> mode_rules;
    for (const auto &mode: modes) {
        for (const auto &i: params) {
            if (i.find(std::string() + CONF_CONTROL_MODES + "/" + mode) ==
                std::string::npos) {
                continue;
            }
            boost::regex e{std::string() + "(?<=" + mode + "/)(\\w+)"};
            boost::smatch w;
            if (!boost::regex_search(i, w, e)) {
                continue;
            }
            std::string dof = w[0]; // dof name

            auto found =
                std::find_if(CONF_DOF_LOOKUP.begin(), CONF_DOF_LOOKUP.end(),
                    [dof](const std::pair<const char *, int> &t) -> bool {
                        return std::strcmp(dof.c_str(),t.first) == 0;
                    }
                );

            if (found != CONF_DOF_LOOKUP.end()) {
                mode_rules[mode].insert(found->second);
            } else {
                throw control_ros_exception(
                        "Unknown freedom name passed '" + dof + "'"
                        "Possible values are "
                       "'x, y, z, roll, pitch, yaw, surge, sway, heave"
                );
            }
        }
    }

    // Loop through all the modes and break them down
    for (const auto &mode: modes) {
        mvp_msgs::ControlMode m;

        m.name = mode;

        m.dofs = std::vector<int>(
            mode_rules[mode].begin(), mode_rules[mode].end());

        for (const auto &dof: mode_rules[mode]) {
            std::string param;
            param += std::string() + CONF_CONTROL_MODES + "/" + mode + "/" +
                     DOFS[dof] + "/";
            if (dof == DOF::X) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_x.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_x.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_x.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_x.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_x.k_i_min, 0);
            } else if (dof == DOF::Y) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_y.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_y.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_y.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_y.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_y.k_i_min, 0);
            } else if (dof == DOF::Z) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_z.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_z.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_z.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_z.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_z.k_i_min, 0);
            } else if (dof == DOF::ROLL) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_roll.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_roll.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_roll.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_roll.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_roll.k_i_min,
                                    0);
            } else if (dof == DOF::PITCH) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_pitch.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_pitch.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_pitch.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_pitch.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_pitch.k_i_min,
                                    0);
            } else if (dof == DOF::YAW) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_yaw.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_yaw.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_yaw.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_yaw.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_yaw.k_i_min,
                                    0);
            } else if (dof == DOF::SURGE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_surge.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_surge.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_surge.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_surge.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_surge.k_i_min,
                                    0);
            } else if (dof == DOF::SWAY) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_sway.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_sway.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_sway.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_sway.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_sway.k_i_min,
                                    0);
            } else if (dof == DOF::HEAVE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_heave.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_heave.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_heave.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX, m.pid_heave.k_i_max,
                                    0);
                m_pnh.param<double>(param + CONF_PID_I_MIN, m.pid_heave.k_i_min,
                                    0);
            } else if (dof == DOF::ROLL_RATE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_roll_rate.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_roll_rate.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_roll_rate.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX,
                                    m.pid_roll_rate.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN,
                                    m.pid_roll_rate.k_i_min, 0);
            } else if (dof == DOF::PITCH_RATE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_pitch_rate.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_pitch_rate.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_pitch_rate.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX,
                                    m.pid_pitch_rate.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN,
                                    m.pid_pitch_rate.k_i_min, 0);
            } else if (dof == DOF::YAW_RATE) {
                m_pnh.param<double>(param + CONF_PID_P, m.pid_yaw_rate.kp, 0);
                m_pnh.param<double>(param + CONF_PID_I, m.pid_yaw_rate.ki, 0);
                m_pnh.param<double>(param + CONF_PID_D, m.pid_yaw_rate.kd, 0);
                m_pnh.param<double>(param + CONF_PID_I_MAX,
                                    m.pid_yaw_rate.k_i_max, 0);
                m_pnh.param<double>(param + CONF_PID_I_MIN,
                                    m.pid_yaw_rate.k_i_min, 0);
            }
        }

        m_control_modes.modes.emplace_back(m);

    }

    f_amend_control_mode(*modes.begin());
}

bool MvpControlROS::f_cb_srv_get_control_modes(
    mvp_msgs::GetControlModes::Request &req,
    mvp_msgs::GetControlModes::Response &resp) {

    if(!m_control_modes.modes.empty()) {
        resp.modes = m_control_modes.modes;
        return true;
    } else {
        return false;
    }

}

bool MvpControlROS::f_cb_srv_set_control_point(
        mvp_msgs::SetControlPoint::Request req,
        mvp_msgs::SetControlPoint::Response resp) {

    return f_amend_set_point(req.setpoint);
}

bool MvpControlROS::f_cb_srv_enable(
        std_srvs::Empty::Request req, std_srvs::Empty::Response res) {

    ROS_INFO("Controller enabled!");
    m_enabled = true;

    return true;
}

bool MvpControlROS::f_cb_srv_disable(
        std_srvs::Empty::Request req, std_srvs::Empty::Response res) {

    ROS_INFO("Controller disabled!");
    m_enabled = false;

    return true;
}

bool MvpControlROS::f_cb_srv_get_controller_state(
        std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    resp.success = true;
    if(m_enabled){
         resp.message = "enabled";
    }
    else{
        resp.message = "disabled";
    }

    return true;
}

bool MvpControlROS::f_cb_srv_get_active_mode(
    mvp_msgs::GetControlMode::Request& req,
    mvp_msgs::GetControlMode::Response& resp) {

    auto found =
        std::find_if(
            m_control_modes.modes.begin(),
            m_control_modes.modes.end(),
            [this](const mvp_msgs::ControlMode &t) -> bool {
                 if(this->m_control_mode == t.name) {
                     return true;
                 }
                 return false;
            }
        );

    resp.mode = *found;

    return true;
}

Eigen::MatrixXd MvpControlROS::f_angular_velocity_transform(const Eigen::VectorXd& orientation) {
    Eigen::Matrix3d transform = Eigen::Matrix3d::Zero();

    // 85 < pitch < 95, -95 < pitch < -85 
    if( (orientation(DOF::PITCH) >  1.483529839 && orientation(DOF::PITCH) <  1.658062761) ||
        (orientation(DOF::PITCH) > -1.658062761 && orientation(DOF::PITCH) < -1.483529839) ) {
        transform(0,0) = 1.0;
        transform(0,1) = 0.0;
        transform(0,2) = 0.0;
        transform(1,0) = 0.0;
        transform(1,1) = cos(orientation(DOF::ROLL));
        transform(1,2) = -sin(orientation(DOF::ROLL));
        transform(2,0) = 0.0;
        transform(2,1) = 0.0;
        transform(2,2) = 0.0;
    }
    else {
        transform(0,0) = 1.0;
        transform(0,1) = sin(orientation(DOF::ROLL)) * tan(orientation(DOF::PITCH));
        transform(0,2) = cos(orientation(DOF::ROLL)) * tan(orientation(DOF::PITCH));
        transform(1,0) = 0.0;
        transform(1,1) = cos(orientation(DOF::ROLL));
        transform(1,2) = -sin(orientation(DOF::ROLL));
        transform(2,0) = 0.0;
        transform(2,1) = sin(orientation(DOF::ROLL)) / cos(orientation(DOF::PITCH));
        transform(2,2) = cos(orientation(DOF::ROLL)) / cos(orientation(DOF::PITCH));
    }    

    return transform;
}

bool MvpControlROS::f_amend_control_mode(std::string mode) {
    if(!mode.empty()) {
        if(mode == m_control_mode) {
            // nothing should change. Operation valid
            return true;
        }

        auto found = std::find_if(
                m_control_modes.modes.begin(),
                m_control_modes.modes.end(),
                [mode](const mvp_msgs::ControlMode& m) -> bool {
                if(m.name == mode) {
                    return true;
                }
                return false;
            }
        );

        if(found == m_control_modes.modes.end()) {
            ROS_WARN_STREAM(
                    "Requested mode [" << mode << "] doesn't exist. "
            );

            // mode doesn't exist. Operation invalid
            return false;
        }

        // setting PID for requested mode
        mvp_control::PIDConfig pid_conf;
        pid_conf.x_p = found->pid_x.kp;
        pid_conf.x_i = found->pid_x.ki;
        pid_conf.x_d = found->pid_x.kd;
        pid_conf.x_i_max = found->pid_x.k_i_max;
        pid_conf.x_i_min = found->pid_x.k_i_min;

        pid_conf.y_p = found->pid_y.kp;
        pid_conf.y_i = found->pid_y.ki;
        pid_conf.y_d = found->pid_y.kd;
        pid_conf.y_i_max = found->pid_y.k_i_max;
        pid_conf.y_i_min = found->pid_y.k_i_min;

        pid_conf.z_p = found->pid_z.kp;
        pid_conf.z_i = found->pid_z.ki;
        pid_conf.z_d = found->pid_z.kd;
        pid_conf.z_i_max = found->pid_z.k_i_max;
        pid_conf.z_i_min = found->pid_z.k_i_min;

        pid_conf.roll_p = found->pid_roll.kp;
        pid_conf.roll_i = found->pid_roll.ki;
        pid_conf.roll_d = found->pid_roll.kd;
        pid_conf.roll_i_max = found->pid_roll.k_i_max;
        pid_conf.roll_i_min = found->pid_roll.k_i_min;

        pid_conf.pitch_p = found->pid_pitch.kp;
        pid_conf.pitch_i = found->pid_pitch.ki;
        pid_conf.pitch_d = found->pid_pitch.kd;
        pid_conf.pitch_i_max = found->pid_pitch.k_i_max;
        pid_conf.pitch_i_min = found->pid_pitch.k_i_min;

        pid_conf.yaw_p = found->pid_yaw.kp;
        pid_conf.yaw_i = found->pid_yaw.ki;
        pid_conf.yaw_d = found->pid_yaw.kd;
        pid_conf.yaw_i_max = found->pid_yaw.k_i_max;
        pid_conf.yaw_i_min = found->pid_yaw.k_i_min;

        pid_conf.surge_p = found->pid_surge.kp;
        pid_conf.surge_i = found->pid_surge.ki;
        pid_conf.surge_d = found->pid_surge.kd;
        pid_conf.surge_i_max = found->pid_surge.k_i_max;
        pid_conf.surge_i_min = found->pid_surge.k_i_min;

        pid_conf.sway_p = found->pid_sway.kp;
        pid_conf.sway_i = found->pid_sway.ki;
        pid_conf.sway_d = found->pid_sway.kd;
        pid_conf.sway_i_max = found->pid_sway.k_i_max;
        pid_conf.sway_i_min = found->pid_sway.k_i_min;

        pid_conf.heave_p = found->pid_heave.kp;
        pid_conf.heave_i = found->pid_heave.ki;
        pid_conf.heave_d = found->pid_heave.kd;
        pid_conf.heave_i_max = found->pid_heave.k_i_max;
        pid_conf.heave_i_min = found->pid_heave.k_i_min;

        pid_conf.roll_rate_p = found->pid_roll_rate.kp;
        pid_conf.roll_rate_i = found->pid_roll_rate.ki;
        pid_conf.roll_rate_d = found->pid_roll_rate.kd;
        pid_conf.roll_rate_i_max = found->pid_roll_rate.k_i_max;
        pid_conf.roll_rate_i_min = found->pid_roll_rate.k_i_min;

        pid_conf.pitch_rate_p = found->pid_pitch_rate.kp;
        pid_conf.pitch_rate_i = found->pid_pitch_rate.ki;
        pid_conf.pitch_rate_d = found->pid_pitch_rate.kd;
        pid_conf.pitch_rate_i_max = found->pid_pitch_rate.k_i_max;
        pid_conf.pitch_rate_i_min = found->pid_pitch_rate.k_i_min;

        pid_conf.yaw_rate_p = found->pid_yaw_rate.kp;
        pid_conf.yaw_rate_i = found->pid_yaw_rate.ki;
        pid_conf.yaw_rate_d = found->pid_yaw_rate.kd;
        pid_conf.yaw_rate_i_max = found->pid_yaw_rate.k_i_max;
        pid_conf.yaw_rate_i_min = found->pid_yaw_rate.k_i_min;

        f_cb_dynconf_pid(pid_conf, 0);

        f_amend_dynconf();

        m_control_mode = mode;

        m_mvp_control->update_freedoms(found->dofs);

        ROS_INFO_STREAM("Controller mode changed to " << mode);

        // mode is not empty. mode is in the modes list. operation is valid.
        return true;
    } else {

        // its empty, operation valid.
        return true;
    }
}

bool MvpControlROS::f_amend_set_point(
    const mvp_msgs::ControlProcess &set_point) {

    if(!f_amend_control_mode(set_point.control_mode)) {
        return false;
    }

    if( set_point.header.frame_id.empty()) {
        // no decision can be made
        ROS_WARN_STREAM("no frame id provided for the setpoint!");
        return false;
    }

    Eigen::Vector3d p_world, rpy_world;
    try {
        // Transform the position of setpoint frame_id to world_link
        auto tf_world_setpoint = m_transform_buffer.lookupTransform(
            m_world_link_id,
            set_point.header.frame_id,
            ros::Time(0)
        );
        if (abs(tf_world_setpoint.header.stamp.toSec() - ros::Time::now().toSec()) < 10 || tf_world_setpoint.header.stamp.toSec()==0.0) 
        { 
            
            auto tf_eigen = tf2::transformToEigen(tf_world_setpoint);

            p_world = tf_eigen.rotation() * 
                                    Eigen::Vector3d(set_point.position.x, set_point.position.y, set_point.position.z)
                                    + tf_eigen.translation();
            ///convert euler angle into a different frame
            ///find the rotation matrix from the set point frame to the desired pose.
            Eigen::Matrix3d R;
            R = Eigen::AngleAxisd(set_point.orientation.z, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(set_point.orientation.y, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(set_point.orientation.x, Eigen::Vector3d::UnitX());
       
            //find rotation matrix from the world link to the setpoint frame.
            auto tf_1 = m_transform_buffer.lookupTransform(
                set_point.header.frame_id,
                m_world_link_id,
                ros::Time(0)
            );

            if (abs(tf_1.header.stamp.toSec() - ros::Time::now().toSec()) < 10 || tf_1.header.stamp.toSec()==0.0) 
            { 
            
                auto tf_1_eigen = tf2::transformToEigen(tf_1);
                //find the total rotation matrix from the world link to the desired pose.
                Eigen::Matrix3d R_set_point =  tf_1_eigen.rotation() *R;
                // Eigen::Vector3d euler_angles = R_set_point.eulerAngles(2,1,0); // ZYX order
                // rpy_world.z() = euler_angles[0];
                // rpy_world.y() = euler_angles[1];
                // rpy_world.x() = euler_angles[2];

                // printf("from eigen: %lf, %lf, %lf\r\n",rpy_world.x(), rpy_world.y(), rpy_world.z());

                //pitch angle
                rpy_world.y() = asin(-R_set_point(2, 0));

                // Calculate yaw (rotation about Z-axis)
                rpy_world.z() = atan2(R_set_point(1, 0), R_set_point(0, 0));

                // Calculate roll (rotation about X-axis)
                rpy_world.x() = atan2(R_set_point(2, 1), R_set_point(2, 2));
                // printf("from code: %lf, %lf, %lf\r\n",rpy_world.x(), rpy_world.y(), rpy_world.z());


            }
            else
            {
                ROS_WARN( "%s to %s TF too old!", set_point.header.frame_id.c_str(), m_world_link_id.c_str() );
                return false;
            }
        }
        else
        {
            ROS_WARN( "%s to %s TF too old!", m_world_link_id.c_str(), set_point.header.frame_id.c_str() );
            return false;
        }
        // rpy_world = tf_eigen.rotation() * 
                                    // Eigen::Vector3d(set_point.orientation.x, set_point.orientation.y, set_point.orientation.z);

        //assume the set point uvw and pqr are in the m_cg_link_id

    } catch(tf2::TransformException &e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string("Can't transform the p and rpy to the global!") + e.what());
        return false;
    }

    m_set_point(mvp_msgs::ControlMode::DOF_X) =
        p_world.x();
    m_set_point(mvp_msgs::ControlMode::DOF_Y) =
        p_world.y();
    m_set_point(mvp_msgs::ControlMode::DOF_Z) =
        p_world.z();
    m_set_point(mvp_msgs::ControlMode::DOF_ROLL) =
        rpy_world.x();
    m_set_point(mvp_msgs::ControlMode::DOF_PITCH) =
        rpy_world.y();
    m_set_point(mvp_msgs::ControlMode::DOF_YAW) =
        rpy_world.z();
    m_set_point(mvp_msgs::ControlMode::DOF_SURGE) =
        set_point.velocity.x;
    m_set_point(mvp_msgs::ControlMode::DOF_SWAY) =
        set_point.velocity.y;
    m_set_point(mvp_msgs::ControlMode::DOF_HEAVE) =
        set_point.velocity.z;
    m_set_point(mvp_msgs::ControlMode::DOF_ROLL_RATE) =
        set_point.angular_rate.x;
    m_set_point(mvp_msgs::ControlMode::DOF_PITCH_RATE) =
        set_point.angular_rate.y;
    m_set_point(mvp_msgs::ControlMode::DOF_YAW_RATE) =
        set_point.angular_rate.z;

    m_mvp_control->update_desired_state(m_set_point);

    m_set_point_msg = set_point;

    return true;
}