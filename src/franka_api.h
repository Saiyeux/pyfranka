
#pragma once

#include <iostream>
#include <functional>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/control_types.h>
#include <eigen3/Eigen/Dense>
#include "franka_log.h"

using namespace Eigen;
using namespace std;

using cartesian_pose_controller_callback = std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>;
using torques_controller_callback = std::function<franka::Torques(const franka::RobotState&, franka::Duration)>;
using joint_velocities_controller_callback = std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>;
using joint_positions_controller_callback = std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>;
using cartesian_velocities_controller_callback = std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)>;

using read_robot_state_callback = std::function<bool(const franka::RobotState&)>;

class FrankaApi
{
public:
    explicit FrankaApi();
    ~FrankaApi();

public:
//    void init_config(const string &ip);
    void init_config(const std::string& ip,
                     franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kEnforce,
                     int log_size = 50);

    void set_default_behavior();
    void set_collision_behavior();

public:
    bool robot_control(torques_controller_callback torques_handle, bool limit_rate = true, double cutoff_frequency = franka::kDefaultCutoffFrequency);

    bool robot_control(torques_controller_callback torques_handle, joint_positions_controller_callback joint_positions_handle,
                       bool limit_rate = true, double cutoff_frequency = franka::kDefaultCutoffFrequency);

    bool robot_control(torques_controller_callback torques_handle, joint_velocities_controller_callback joint_velocities_handle,
                       bool limit_rate = true, double cutoff_frequency = franka::kDefaultCutoffFrequency);

    bool robot_control(torques_controller_callback torques_handle, cartesian_pose_controller_callback cartesian_pose_handle,
                       bool limit_rate = true, double cutoff_frequency = franka::kDefaultCutoffFrequency);

    bool robot_control(torques_controller_callback torques_handle, cartesian_velocities_controller_callback cartesian_velocities_handle,
                       bool limit_rate = true, double cutoff_frequency = franka::kDefaultCutoffFrequency);

    bool robot_control(joint_positions_controller_callback joint_positions_handle, franka::ControllerMode controller_mode = franka::ControllerMode::kJointImpedance,
                       bool limit_rate = true, double cutoff_frequency = franka::kDefaultCutoffFrequency);

    bool robot_control(joint_velocities_controller_callback joint_velocities_handle, franka::ControllerMode controller_mode = franka::ControllerMode::kJointImpedance,
                       bool limit_rate = true, double cutoff_frequency = franka::kDefaultCutoffFrequency);

    bool robot_control(cartesian_pose_controller_callback cartesian_pose_handle, franka::ControllerMode controller_mode = franka::ControllerMode::kJointImpedance,
                       bool limit_rate = true, double cutoff_frequency = franka::kDefaultCutoffFrequency);

    bool robot_control(cartesian_velocities_controller_callback cartesian_velocities_handle, franka::ControllerMode controller_mode = franka::ControllerMode::kJointImpedance,
                       bool limit_rate = true, double cutoff_frequency = franka::kDefaultCutoffFrequency);

    // Cartesian velocity callback → joint velocity control with j7 null-space pinning.
    // Uses libfranka model Jacobian (accurate, runs fully in C++, no Python overhead).
    bool robot_control_j7_pinned(
        cartesian_velocities_controller_callback cv_handle,
        double q7_target,
        double j7_gain = 1.5,
        double dq_max  = 0.3,
        bool   limit_rate = true,
        double cutoff_frequency = franka::kDefaultCutoffFrequency);

    bool read_robot_state(read_robot_state_callback read_robot_state_handle);

    void set_collision_behavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                                const std::array<double, 7>& upper_torque_thresholds_acceleration,
                                const std::array<double, 7>& lower_torque_thresholds_nominal,
                                const std::array<double, 7>& upper_torque_thresholds_nominal,
                                const std::array<double, 6>& lower_force_thresholds_acceleration,
                                const std::array<double, 6>& upper_force_thresholds_acceleration,
                                const std::array<double, 6>& lower_force_thresholds_nominal,
                                const std::array<double, 6>& upper_force_thresholds_nominal);

    void set_collision_behavior(const std::array<double, 7>& lower_torque_thresholds,
                              const std::array<double, 7>& upper_torque_thresholds,
                              const std::array<double, 6>& lower_force_thresholds,
                              const std::array<double, 6>& upper_force_thresholds);

    void automatic_error_recovery();

    void set_joint_impedance(const std::array<double, 7>& K_theta);

    void set_cartesian_impedance(const std::array<double, 6>& K_x);

    void set_guiding_mode(const std::array<bool, 6>& guiding_mode, bool elbow);

    void setK(const std::array<double, 16>& EE_T_K);

    void setEE(const std::array<double, 16>& NE_T_EE);

    void setLoad(double load_mass,
                 const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
                 const std::array<double, 9>& load_inertia);

public:
    franka::Model loadModel();

    franka::RobotState readOnce();

    void stop();


public:
    void request_robot_state();

public:
    void test(const string &ip = "localhost");

private:
    std::unique_ptr<franka::Robot> robot;
//    franka::Robot *robot;

};


