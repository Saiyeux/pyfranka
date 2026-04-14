
#include "franka_api.h"
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

void FrankaApi::test(const string &ip)
{
    cout << "ip:" << ip << endl;
}

