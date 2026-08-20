
#include "franka_api.h"
#include "j7_pinned_solver.h"
#include <algorithm>
#include <array>
#include <cmath>

FrankaApi::FrankaApi()
{
}

FrankaApi::~FrankaApi()
{
    robot.release();
}

//void FrankaApi::init_config(const string &ip)
//{
//    try {
//        robot = std::make_unique<franka::Robot>(ip);
////	    robot = new franka::Robot(ip);
//    }
//    catch (franka::Exception const& e)
//    {
//        std::cout << e.what() << std::endl;
//    }
//}

void FrankaApi::init_config(const std::string &ip, franka::RealtimeConfig realtime_config, int log_size)
{
    try {
        robot = std::make_unique<franka::Robot>(ip, realtime_config, static_cast<std::size_t>(log_size));
    }
    catch (franka::Exception const& e)
    {
        std::cout << e.what() << std::endl;
    }
}


void FrankaApi::set_default_behavior()
{
    try {
        robot->setCollisionBehavior(
                {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    }
    catch (franka::Exception const& e)
    {
        std::cout << e.what() << std::endl;
    }
}

void FrankaApi::set_collision_behavior()
{
    robot->setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
}

franka::Model FrankaApi::loadModel()
{
    return robot->loadModel();
}

franka::RobotState FrankaApi::readOnce()
{
    return robot->readOnce();
}

void FrankaApi::stop()
{
    robot->stop();
}

void FrankaApi::request_robot_state()
{
    try {
        size_t count = 0;
        robot->read([&count](const franka::RobotState& robot_state) {
            // Printing to std::cout adds a delay. This is acceptable for a read loop such as this, but
            // should not be done in a control loop.
            std::cout << robot_state << std::endl;
            return count++ < 100;
        });
    }
    catch (franka::Exception const& e)
    {
        std::cout << e.what() << std::endl;
    }
}

bool FrankaApi::robot_control(torques_controller_callback torques_handle, bool limit_rate, double cutoff_frequency)
{
    try {
        if (torques_handle != nullptr){
            robot->control(torques_handle, limit_rate, cutoff_frequency);

            cout << "Finished motion." << std::endl;
            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

bool FrankaApi::robot_control(torques_controller_callback torques_handle, joint_positions_controller_callback joint_positions_handle,
                   bool limit_rate, double cutoff_frequency)
{
    try {
        if ((torques_handle != nullptr) && (joint_positions_handle != nullptr)){
            robot->control(torques_handle, joint_positions_handle, limit_rate, cutoff_frequency);

            cout << "Finished motion." << std::endl;
            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

bool FrankaApi::robot_control(torques_controller_callback torques_handle, joint_velocities_controller_callback joint_velocities_handle,
                   bool limit_rate, double cutoff_frequency)
{
    try {
        if ((torques_handle != nullptr) && (joint_velocities_handle != nullptr)){
            robot->control(torques_handle, joint_velocities_handle, limit_rate, cutoff_frequency);

            cout << "Finished motion." << std::endl;
            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

bool FrankaApi::robot_control(torques_controller_callback torques_handle, cartesian_pose_controller_callback cartesian_pose_handle,
                   bool limit_rate, double cutoff_frequency)
{
    try {
        if ((torques_handle != nullptr) && (cartesian_pose_handle != nullptr)){
            robot->control(torques_handle, cartesian_pose_handle, limit_rate, cutoff_frequency);

            cout << "Finished motion." << std::endl;
            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

bool FrankaApi::robot_control(torques_controller_callback torques_handle, cartesian_velocities_controller_callback cartesian_velocities_handle,
                   bool limit_rate, double cutoff_frequency)
{
    try {
        if ((torques_handle != nullptr) && (cartesian_velocities_handle != nullptr)){
            robot->control(torques_handle, cartesian_velocities_handle, limit_rate, cutoff_frequency);

            cout << "Finished motion." << std::endl;
            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

bool FrankaApi::robot_control(joint_positions_controller_callback joint_positions_handle, franka::ControllerMode controller_mode,
                   bool limit_rate, double cutoff_frequency)
{
    try {
        if (joint_positions_handle != nullptr){
            robot->control(joint_positions_handle, controller_mode, limit_rate, cutoff_frequency);

            cout << "Finished motion." << std::endl;
            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

bool FrankaApi::robot_control(joint_velocities_controller_callback joint_velocities_handle, franka::ControllerMode controller_mode,
                   bool limit_rate, double cutoff_frequency)
{
    try {
        if (joint_velocities_handle != nullptr){
            robot->control(joint_velocities_handle, controller_mode, limit_rate, cutoff_frequency);

            cout << "Finished motion." << std::endl;
            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

bool FrankaApi::robot_control(cartesian_pose_controller_callback cartesian_pose_handle, franka::ControllerMode controller_mode,
                   bool limit_rate, double cutoff_frequency)
{
    try {
        if (cartesian_pose_handle != nullptr){
            robot->control(cartesian_pose_handle, controller_mode, limit_rate, cutoff_frequency);

            cout << "Finished motion." << std::endl;
            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

bool FrankaApi::robot_control(cartesian_velocities_controller_callback cartesian_velocities_handle, franka::ControllerMode controller_mode,
                   bool limit_rate, double cutoff_frequency)
{
    try {
        if (cartesian_velocities_handle != nullptr){
            robot->control(cartesian_velocities_handle, controller_mode, limit_rate, cutoff_frequency);

            cout << "Finished motion." << std::endl;
            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

bool FrankaApi::robot_control_cartesian_velocities_posture(
    cartesian_velocities_controller_callback cv_handle,
    double posture_gain,
    double dq_max,
    bool limit_rate,
    double cutoff_frequency)
{
    const double lam = 0.02;
    const double joint_limit_margin = 0.10;

    franka::Model model = robot->loadModel();
    Eigen::Matrix<double, 7, 1> q_reference = Eigen::Matrix<double, 7, 1>::Zero();
    bool have_reference = false;
    bool stopped_on_joint_limit = false;
    int stopped_joint = -1;
    double stopped_margin = 0.0;

    try {
        robot->control([&](const franka::RobotState& rs, franka::Duration period) -> franka::JointVelocities {
            int limit_joint = -1;
            const double limit_margin = j7_pinned::min_joint_margin(rs.q, &limit_joint);
            if (limit_margin < joint_limit_margin) {
                stopped_on_joint_limit = true;
                stopped_joint = limit_joint;
                stopped_margin = limit_margin;
                return franka::MotionFinished(franka::JointVelocities({0,0,0,0,0,0,0}));
            }

            Eigen::Matrix<double, 7, 1> q;
            for (int i = 0; i < 7; i++) q(i) = rs.q[i];
            if (!have_reference) {
                q_reference = q;
                have_reference = true;
            }

            franka::CartesianVelocities cv = cv_handle(rs, period);
            if (cv.motion_finished) {
                return franka::MotionFinished(franka::JointVelocities({0,0,0,0,0,0,0}));
            }

            std::array<double, 42> J_flat = model.zeroJacobian(franka::Frame::kEndEffector, rs);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_flat.data());

            Eigen::Matrix<double, 6, 1> v;
            for (int i = 0; i < 6; i++) v(i) = cv.O_dP_EE[i];

            // Transit/contact/lift must keep the brush axis safe, but yaw
            // around the brush axis is irrelevant.  Track XYZ + two tilt axes;
            // any posture hold remains optional and is passed in by Python.
            Eigen::Map<const Eigen::Matrix<double, 4, 4>> O_T_EE(rs.O_T_EE.data());
            const Eigen::Vector3d tool_axis =
                j7_pinned::normalized_or_z(O_T_EE.block<3, 1>(0, 2));
            Eigen::Vector3d u;
            Eigen::Vector3d w;
            j7_pinned::perpendicular_basis(tool_axis, &u, &w);

            Eigen::Matrix<double, 5, 7> M = Eigen::Matrix<double, 5, 7>::Zero();
            M.block<3, 7>(0, 0) = J.block<3, 7>(0, 0);
            M.row(3) = u.transpose() * J.block<3, 7>(3, 0);
            M.row(4) = w.transpose() * J.block<3, 7>(3, 0);

            Eigen::Matrix<double, 5, 1> b = Eigen::Matrix<double, 5, 1>::Zero();
            b.head<3>() = v.head<3>();
            b(3) = u.dot(v.tail<3>());
            b(4) = w.dot(v.tail<3>());

            Eigen::Matrix<double, 7, 5> M_pinv =
                M.transpose() *
                (M * M.transpose() + lam * lam * Eigen::Matrix<double, 5, 5>::Identity()).inverse();

            Eigen::Matrix<double, 7, 1> dq = M_pinv * b;
            const Eigen::Matrix<double, 7, 7> N =
                Eigen::Matrix<double, 7, 7>::Identity() - M_pinv * M;
            dq += N * (posture_gain * (q_reference - q));

            for (int i = 0; i < 7; i++)
                dq(i) = std::max(-dq_max, std::min(dq_max, dq(i)));

            return franka::JointVelocities(
                {dq(0), dq(1), dq(2), dq(3), dq(4), dq(5), dq(6)});

        }, franka::ControllerMode::kJointImpedance, limit_rate, cutoff_frequency);

        cout << "Finished motion." << std::endl;
        if (stopped_on_joint_limit) {
            std::cout << "Posture-stable Cartesian motion stopped near joint limit: J"
                      << (stopped_joint + 1)
                      << " margin=" << stopped_margin << " rad" << std::endl;
            return false;
        }
        return true;
    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }
}

bool FrankaApi::read_robot_state(read_robot_state_callback read_robot_state_handle)
{
    try {
        if (read_robot_state_handle != nullptr){
            robot->read(read_robot_state_handle);

            return true;
        }

    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }

    return true;
}

void FrankaApi::set_collision_behavior(const std::array<double, 7>& lower_torque_thresholds_acceleration,
                            const std::array<double, 7>& upper_torque_thresholds_acceleration,
                            const std::array<double, 7>& lower_torque_thresholds_nominal,
                            const std::array<double, 7>& upper_torque_thresholds_nominal,
                            const std::array<double, 6>& lower_force_thresholds_acceleration,
                            const std::array<double, 6>& upper_force_thresholds_acceleration,
                            const std::array<double, 6>& lower_force_thresholds_nominal,
                            const std::array<double, 6>& upper_force_thresholds_nominal)
{
    robot->setCollisionBehavior(lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
                                lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                                lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
                                lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

void FrankaApi::set_collision_behavior(const std::array<double, 7>& lower_torque_thresholds,
                            const std::array<double, 7>& upper_torque_thresholds,
                            const std::array<double, 6>& lower_force_thresholds,
                            const std::array<double, 6>& upper_force_thresholds)
{
    robot->setCollisionBehavior(lower_torque_thresholds, upper_torque_thresholds, lower_force_thresholds, upper_force_thresholds);
}

void FrankaApi::automatic_error_recovery()
{
    robot->automaticErrorRecovery();
}

void FrankaApi::set_joint_impedance(const std::array<double, 7>& K_theta)
{
    robot->setJointImpedance(K_theta);
}

void FrankaApi::set_cartesian_impedance(const std::array<double, 6>& K_x)
{
    robot->setCartesianImpedance(K_x);
}

void FrankaApi::set_guiding_mode(const std::array<bool, 6>& guiding_mode, bool elbow)
{
    robot->setGuidingMode(guiding_mode, elbow);
}

void FrankaApi::setK(const std::array<double, 16>& EE_T_K)
{
    robot->setK(EE_T_K);
}

void FrankaApi::setEE(const std::array<double, 16>& NE_T_EE)
{
    robot->setEE(NE_T_EE);
}

void FrankaApi::setLoad(double load_mass,
             const std::array<double, 3>& F_x_Cload,  // NOLINT(readability-identifier-naming)
             const std::array<double, 9>& load_inertia)
{
    robot->setLoad(load_mass, F_x_Cload, load_inertia);
}

bool FrankaApi::robot_control_j7_pinned(
    cartesian_velocities_controller_callback cv_handle,
    double q7_target,
    double j7_gain,
    double dq_max,
    bool   limit_rate,
    double cutoff_frequency)
{
    const double lam = 0.02;  // DLS damping: velocity tracking with singularity safety.
    const double posture_gain = 0.3;
    const double joint_limit_margin = 0.10;

    franka::Model model = robot->loadModel();
    Eigen::Matrix<double, 7, 1> q_reference = Eigen::Matrix<double, 7, 1>::Zero();
    bool have_reference = false;
    bool stopped_on_joint_limit = false;
    int stopped_joint = -1;
    double stopped_margin = 0.0;

    try {
        robot->control([&](const franka::RobotState& rs, franka::Duration period) -> franka::JointVelocities {
            int limit_joint = -1;
            const double limit_margin = j7_pinned::min_joint_margin(rs.q, &limit_joint);
            if (limit_margin < joint_limit_margin) {
                stopped_on_joint_limit = true;
                stopped_joint = limit_joint;
                stopped_margin = limit_margin;
                return franka::MotionFinished(franka::JointVelocities({0,0,0,0,0,0,0}));
            }

            Eigen::Matrix<double, 7, 1> q;
            for (int i = 0; i < 7; i++) q(i) = rs.q[i];
            if (!have_reference) {
                q_reference = q;
                q_reference(6) = q7_target;
                have_reference = true;
            }

            // Ask Python callback for desired Cartesian velocity
            franka::CartesianVelocities cv = cv_handle(rs, period);

            if (cv.motion_finished) {
                return franka::MotionFinished(franka::JointVelocities({0,0,0,0,0,0,0}));
            }

            // Zero Jacobian: 6×7, column-major, 42 doubles
            std::array<double, 42> J_flat = model.zeroJacobian(franka::Frame::kEndEffector, rs);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> J(J_flat.data());

            // Desired Cartesian velocity
            Eigen::Matrix<double, 6, 1> v;
            for (int i = 0; i < 6; i++) v(i) = cv.O_dP_EE[i];

            // Painting with a round brush does not need yaw about the brush axis.
            // Track XYZ + tool-axis tilt + J7, then use the remaining null-space
            // to stay near the stroke-start posture.
            Eigen::Map<const Eigen::Matrix<double, 4, 4>> O_T_EE(rs.O_T_EE.data());
            const Eigen::Vector3d tool_axis = O_T_EE.block<3, 1>(0, 2);
            Eigen::Matrix<double, 7, 1> dq = j7_pinned::solve_velocity(
                J, v, tool_axis, q, q_reference, q7_target, j7_gain,
                posture_gain, lam);

            // Hard velocity limit
            for (int i = 0; i < 7; i++)
                dq(i) = std::max(-dq_max, std::min(dq_max, dq(i)));

            return franka::JointVelocities(
                {dq(0), dq(1), dq(2), dq(3), dq(4), dq(5), dq(6)});

        }, franka::ControllerMode::kJointImpedance, limit_rate, cutoff_frequency);

        cout << "Finished motion." << std::endl;
        if (stopped_on_joint_limit) {
            std::cout << "J7-pinned motion stopped near joint limit: J"
                      << (stopped_joint + 1)
                      << " margin=" << stopped_margin << " rad" << std::endl;
            return false;
        }
        return true;
    }
    catch (const franka::ControlException& e) {
        std::cout << e.what() << std::endl;
        writeLogToFile(e.log);
        std::cout << "Running error recovery..." << std::endl;
        robot->automaticErrorRecovery();
        return false;
    }
}

void FrankaApi::test(const string &ip)
{
    cout << "ip:" << ip << endl;
}
