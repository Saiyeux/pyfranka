
#include <pybind11/pybind11.h>
#include <pybind11/complex.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include <pybind11/cast.h>
#include <cstdint>
#include <ratio>
#include <iostream>
#include <initializer_list>

#include "franka_api.h"
#include "motion_generator.h"
#include "franka_control_types.h"

#include "franka/robot_state.h"
#include "franka/errors.h"
#include "franka/duration.h"
#include "franka/control_types.h"
#include "franka/gripper_state.h"
#include "franka/gripper.h"

//#include "franka_log.h"

namespace py = pybind11;

PYBIND11_MODULE(franka_pybind, m) {

    py::enum_<franka::ControllerMode>(m, "ControllerMode")
            .value("kJointImpedance", franka::ControllerMode::kJointImpedance)
            .value("kCartesianImpedance", franka::ControllerMode::kCartesianImpedance)
            .export_values();

    py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig")
            .value("kEnforce", franka::RealtimeConfig::kEnforce)
            .value("kIgnore", franka::RealtimeConfig::kIgnore)
            .export_values();

    py::class_<FrankaApi>(m, "FrankaApi")
            .def(py::init())

//            .def("init_config", py::overload_cast<const string &>(&FrankaApi::init_config), py::arg("ip"))
//            .def("init_config", py::overload_cast<const string &, franka::RealtimeConfig, int>(&FrankaApi::init_config),
//                 py::arg("ip"), py::arg("realtime_config") = franka::RealtimeConfig::kEnforce, py::arg("log_size") = 50)

            .def("init_config", &FrankaApi::init_config, py::arg("ip"), py::arg("realtime_config") = franka::RealtimeConfig::kEnforce, py::arg("log_size") = 50)

            .def("set_default_behavior", &FrankaApi::set_default_behavior)

            .def("request_robot_state", &FrankaApi::request_robot_state)

                    ////////////////////////
            .def("robot_control", py::overload_cast<torques_controller_callback, bool, double>(&FrankaApi::robot_control),
                    py::arg("torques_handle"), py::arg("limit_rate") = true, py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)

            .def("robot_control", py::overload_cast<torques_controller_callback, joint_positions_controller_callback, bool, double>(&FrankaApi::robot_control),
                 py::arg("torques_handle"), py::arg("joint_positions_handle"), py::arg("limit_rate") = true, py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)

            .def("robot_control", py::overload_cast<torques_controller_callback, joint_velocities_controller_callback, bool, double>(&FrankaApi::robot_control),
                 py::arg("torques_handle"), py::arg("joint_velocities_handle"), py::arg("limit_rate") = true, py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)

            .def("robot_control", py::overload_cast<torques_controller_callback, cartesian_pose_controller_callback, bool, double>(&FrankaApi::robot_control),
                 py::arg("torques_handle"), py::arg("cartesian_pose_handle"), py::arg("limit_rate") = true, py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)

            .def("robot_control", py::overload_cast<torques_controller_callback, cartesian_velocities_controller_callback, bool, double>(&FrankaApi::robot_control),
                 py::arg("torques_handle"), py::arg("cartesian_velocities_handle"), py::arg("limit_rate") = true, py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)

            .def("robot_control", py::overload_cast<joint_positions_controller_callback, franka::ControllerMode, bool, double>(&FrankaApi::robot_control),
                 py::arg("joint_positions_handle"), py::arg("controller_mode")= franka::ControllerMode::kJointImpedance,
                 py::arg("limit_rate") = true, py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)

            .def("robot_control", py::overload_cast<joint_velocities_controller_callback, franka::ControllerMode, bool, double>(&FrankaApi::robot_control),
                 py::arg("joint_velocities_handle"), py::arg("controller_mode")= franka::ControllerMode::kJointImpedance,
                 py::arg("limit_rate") = true, py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)

            .def("robot_control", py::overload_cast<cartesian_pose_controller_callback, franka::ControllerMode, bool, double>(&FrankaApi::robot_control),
                 py::arg("cartesian_pose_handle"), py::arg("controller_mode")= franka::ControllerMode::kJointImpedance,
                 py::arg("limit_rate") = true, py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)

            .def("robot_control", py::overload_cast<cartesian_velocities_controller_callback, franka::ControllerMode, bool, double>(&FrankaApi::robot_control),
                 py::arg("cartesian_velocities_handle"), py::arg("controller_mode")= franka::ControllerMode::kJointImpedance,
                 py::arg("limit_rate") = true, py::arg("cutoff_frequency") = franka::kDefaultCutoffFrequency)
                    ////////////////////////
            .def("read_robot_state", &FrankaApi::read_robot_state)
                    ////////////////////////
            .def("set_collision_behavior", py::overload_cast<>(&FrankaApi::set_collision_behavior))
            .def("set_collision_behavior", py::overload_cast<const std::array<double, 7>&, const std::array<double, 7>&,
                    const std::array<double, 7>&, const std::array<double, 7>&,
                    const std::array<double, 6>&, const std::array<double, 6>&,
                    const std::array<double, 6>&, const std::array<double, 6>&>(&FrankaApi::set_collision_behavior))

            .def("set_collision_behavior", py::overload_cast<const std::array<double, 7>&, const std::array<double, 7>&,
                    const std::array<double, 6>&, const std::array<double, 6>&>(&FrankaApi::set_collision_behavior))
                    ////////////////////////
            .def("automatic_error_recovery", &FrankaApi::automatic_error_recovery)

            .def("set_joint_impedance", &FrankaApi::set_joint_impedance)
            .def("set_cartesian_impedance", &FrankaApi::set_cartesian_impedance)
            .def("set_guiding_mode", &FrankaApi::set_guiding_mode)
            .def("setK", &FrankaApi::setK)
            .def("setEE", &FrankaApi::setEE)
            .def("setLoad", &FrankaApi::setLoad)

            .def("loadModel", &FrankaApi::loadModel)
            .def("readOnce", &FrankaApi::readOnce)
            .def("stop", &FrankaApi::stop)

            .def("test", &FrankaApi::test, py::arg("ip") = "localhost")

            ;

    py::class_<franka::Model>(m, "Model")
//            .def(py::init<franka::Network& >())

            .def("pose", py::overload_cast<franka::Frame, const franka::RobotState&>(&franka::Model::pose, pybind11::const_))

            .def("pose", py::overload_cast<franka::Frame, const std::array<double, 7>&,
                    const std::array<double, 16>&, const std::array<double, 16>&>(&franka::Model::pose, pybind11::const_))

            .def("bodyJacobian", py::overload_cast<franka::Frame, const franka::RobotState&>(&franka::Model::bodyJacobian, pybind11::const_))

            .def("bodyJacobian", py::overload_cast<franka::Frame, const std::array<double, 7>&,
                    const std::array<double, 16>&, const std::array<double, 16>&>(&franka::Model::bodyJacobian, pybind11::const_))

            .def("zeroJacobian", py::overload_cast<franka::Frame, const franka::RobotState&>(&franka::Model::zeroJacobian, py::const_))

            .def("zeroJacobian", py::overload_cast<franka::Frame, const std::array<double, 7>&, const std::array<double, 16>&,
                    const std::array<double, 16>&>(&franka::Model::zeroJacobian, py::const_))

            .def("mass", py::overload_cast<const franka::RobotState&>(&franka::Model::mass, pybind11::const_))
            .def("mass", py::overload_cast<const std::array<double, 7>&, const std::array<double, 9>&,
                    double, const std::array<double, 3>&>(&franka::Model::mass, pybind11::const_))

            .def("coriolis", py::overload_cast<const franka::RobotState& >(&franka::Model::coriolis, py::const_))
            .def("coriolis", py::overload_cast<const std::array<double, 7>&, const std::array<double, 7>&, const std::array<double, 9>&,
                    double, const std::array<double, 3>&>(&franka::Model::coriolis, py::const_))

            .def("gravity", py::overload_cast<const std::array<double, 7>&, double, const std::array<double, 3>&,
                    const std::array<double, 3>& >(&franka::Model::gravity, pybind11::const_))

            .def("gravity", py::overload_cast<const franka::RobotState&, const std::array<double, 3>&>(&franka::Model::gravity, py::const_))

            ;

//    py::class_<franka::Network>(m, "Network")
//            .def(py::init<const std::string&, uint16_t, std::chrono::milliseconds, std::chrono::milliseconds,
//                    std::tuple<bool, int, int, int>>());


    py::enum_<franka::Frame>(m, "Frame")
            .value("kJoint1", franka::Frame::kJoint1)
            .value("kJoint2", franka::Frame::kJoint2)
            .value("kJoint3", franka::Frame::kJoint3)
            .value("kJoint4", franka::Frame::kJoint4)
            .value("kJoint5", franka::Frame::kJoint5)
            .value("kJoint6", franka::Frame::kJoint6)
            .value("kJoint7", franka::Frame::kJoint7)
            .value("kFlange", franka::Frame::kFlange)
            .value("kEndEffector", franka::Frame::kEndEffector)
            .value("kStiffness", franka::Frame::kStiffness)
            .export_values();



    // py::class_<MotionGenerator>(m, "MotionGenerator")
    // 	.def(py::init<double, const std::array<double, 7>>());

    py::class_<franka::Errors>(m, "Errors")
            .def(py::init<>())
            .def(py::init<const franka::Errors&>())
            .def(py::init<const std::array<bool, 41>&>())

            .def_property_readonly("joint_position_limits_violation", [](const franka::Errors& e) { return e.joint_position_limits_violation; })
            .def_property_readonly("cartesian_position_limits_violation", [](const franka::Errors& e) { return e.cartesian_position_limits_violation; })
            .def_property_readonly("self_collision_avoidance_violation", [](const franka::Errors& e) { return e.self_collision_avoidance_violation; })
            .def_property_readonly("joint_velocity_violation", [](const franka::Errors& e) { return e.joint_velocity_violation; })
            .def_property_readonly("cartesian_velocity_violation", [](const franka::Errors& e) { return e.cartesian_velocity_violation; })
            .def_property_readonly("force_control_safety_violation", [](const franka::Errors& e) { return e.force_control_safety_violation; })
            .def_property_readonly("joint_reflex", [](const franka::Errors& e) { return e.joint_reflex; })
            .def_property_readonly("cartesian_reflex", [](const franka::Errors& e) { return e.cartesian_reflex; })
            .def_property_readonly("max_goal_pose_deviation_violation", [](const franka::Errors& e) { return e.max_goal_pose_deviation_violation; })
            .def_property_readonly("max_path_pose_deviation_violation", [](const franka::Errors& e) { return e.max_path_pose_deviation_violation; })
            .def_property_readonly("cartesian_velocity_profile_safety_violation", [](const franka::Errors& e) { return e.cartesian_velocity_profile_safety_violation; })
            .def_property_readonly("joint_position_motion_generator_start_pose_invalid", [](const franka::Errors& e) { return e.joint_position_motion_generator_start_pose_invalid; })
            .def_property_readonly("joint_motion_generator_position_limits_violation", [](const franka::Errors& e) { return e.joint_motion_generator_position_limits_violation; })
            .def_property_readonly("joint_motion_generator_velocity_limits_violation", [](const franka::Errors& e) { return e.joint_motion_generator_velocity_limits_violation; })
            .def_property_readonly("joint_motion_generator_velocity_discontinuity", [](const franka::Errors& e) { return e.joint_motion_generator_velocity_discontinuity; })
            .def_property_readonly("joint_motion_generator_acceleration_discontinuity", [](const franka::Errors& e) { return e.joint_motion_generator_acceleration_discontinuity; })
            .def_property_readonly("cartesian_position_motion_generator_start_pose_invalid", [](const franka::Errors& e) { return e.cartesian_position_motion_generator_start_pose_invalid; })
            .def_property_readonly("cartesian_motion_generator_elbow_limit_violation", [](const franka::Errors& e) { return e.cartesian_motion_generator_elbow_limit_violation; })
            .def_property_readonly("cartesian_motion_generator_velocity_limits_violation", [](const franka::Errors& e) { return e.cartesian_motion_generator_velocity_limits_violation; })
            .def_property_readonly("cartesian_motion_generator_velocity_discontinuity", [](const franka::Errors& e) { return e.cartesian_motion_generator_velocity_discontinuity; })
            .def_property_readonly("cartesian_motion_generator_acceleration_discontinuity", [](const franka::Errors& e) { return e.cartesian_motion_generator_acceleration_discontinuity; })
            .def_property_readonly("cartesian_motion_generator_elbow_sign_inconsistent", [](const franka::Errors& e) { return e.cartesian_motion_generator_elbow_sign_inconsistent; })
            .def_property_readonly("cartesian_motion_generator_start_elbow_invalid", [](const franka::Errors& e) { return e.cartesian_motion_generator_start_elbow_invalid; })
            .def_property_readonly("cartesian_motion_generator_joint_position_limits_violation", [](const franka::Errors& e) { return e.cartesian_motion_generator_joint_position_limits_violation; })
            .def_property_readonly("cartesian_motion_generator_joint_velocity_limits_violation", [](const franka::Errors& e) { return e.cartesian_motion_generator_joint_velocity_limits_violation; })
            .def_property_readonly("cartesian_motion_generator_joint_velocity_discontinuity", [](const franka::Errors& e) { return e.cartesian_motion_generator_joint_velocity_discontinuity; })
            .def_property_readonly("cartesian_motion_generator_joint_acceleration_discontinuity", [](const franka::Errors& e) { return e.cartesian_motion_generator_joint_acceleration_discontinuity; })
            .def_property_readonly("cartesian_position_motion_generator_invalid_frame", [](const franka::Errors& e) { return e.cartesian_position_motion_generator_invalid_frame; })
            .def_property_readonly("force_controller_desired_force_tolerance_violation", [](const franka::Errors& e) { return e.force_controller_desired_force_tolerance_violation; })
            .def_property_readonly("controller_torque_discontinuity", [](const franka::Errors& e) { return e.controller_torque_discontinuity; })
            .def_property_readonly("start_elbow_sign_inconsistent", [](const franka::Errors& e) { return e.start_elbow_sign_inconsistent; })
            .def_property_readonly("communication_constraints_violation", [](const franka::Errors& e) { return e.communication_constraints_violation; })
            .def_property_readonly("power_limit_violation", [](const franka::Errors& e) { return e.power_limit_violation; })
            .def_property_readonly("joint_p2p_insufficient_torque_for_planning", [](const franka::Errors& e) { return e.joint_p2p_insufficient_torque_for_planning; })
            .def_property_readonly("tau_j_range_violation", [](const franka::Errors& e) { return e.tau_j_range_violation; })
            .def_property_readonly("instability_detected", [](const franka::Errors& e) { return e.instability_detected; })
            .def_property_readonly("joint_move_in_wrong_direction", [](const franka::Errors& e) { return e.joint_move_in_wrong_direction; })

            .def_property_readonly("errors_bool", [](const franka::Errors &e) {return static_cast<bool>(e); })
            .def_property_readonly("errors_string", [](const franka::Errors &e) {return static_cast<std::string>(e); })
            ;


    py::class_<franka::Duration>(m, "Duration")
            .def(py::init<>())
            .def(py::init<uint64_t>())
            .def(py::init<std::chrono::duration<uint64_t, std::milli>>())
            .def(py::init<const franka::Duration&>())
            .def(py::self == py::self)
                    // operator std::chrono::duration<uint64_t, std::milli>() const noexcept;
            .def("toSec", &franka::Duration::toSec)
            .def("toMSec", &franka::Duration::toMSec)
            .def(py::self + py::self)
            .def(py::self += py::self)
            .def(py::self - py::self)
            .def(py::self -= py::self)
            .def(py::self * uint64_t())
            .def(py::self *= uint64_t())
                    // uint64_t operator/(const Duration& rhs) const noexcept;
            .def(py::self / uint64_t())
            .def(py::self /= uint64_t())
            .def(py::self %= py::self)
            .def(py::self % py::self)
            .def(py::self %= uint64_t())
            .def(py::self % uint64_t());

    py::enum_<franka::RobotMode>(m, "RobotMode")
            .value("kOther", franka::RobotMode::kOther)
            .value("kIdle", franka::RobotMode::kIdle)
            .value("kMove", franka::RobotMode::kMove)
            .value("kGuiding", franka::RobotMode::kGuiding)
            .value("kReflex", franka::RobotMode::kReflex)
            .value("kUserStopped", franka::RobotMode::kUserStopped)
            .value("kAutomaticErrorRecovery", franka::RobotMode::kAutomaticErrorRecovery)
            .export_values();

    py::class_<franka::RobotState>(m, "RobotState")
            .def(py::init())
            .def_readonly("O_T_EE", &franka::RobotState::O_T_EE)
            .def_readonly("O_T_EE_d", &franka::RobotState::O_T_EE_d)
            .def_readonly("F_T_EE", &franka::RobotState::F_T_EE)
            .def_readonly("F_T_NE", &franka::RobotState::F_T_NE)
            .def_readonly("NE_T_EE", &franka::RobotState::NE_T_EE)
            .def_readonly("EE_T_K", &franka::RobotState::EE_T_K)
            .def_readonly("m_ee", &franka::RobotState::m_ee)
            .def_readonly("I_ee", &franka::RobotState::I_ee)
            .def_readonly("F_x_Cee", &franka::RobotState::F_x_Cee)
            .def_readonly("m_load", &franka::RobotState::m_load)
            .def_readonly("I_load", &franka::RobotState::I_load)
            .def_readonly("F_x_Cload", &franka::RobotState::F_x_Cload)
            .def_readonly("m_total", &franka::RobotState::m_total)
            .def_readonly("I_total", &franka::RobotState::I_total)
            .def_readonly("F_x_Ctotal", &franka::RobotState::F_x_Ctotal)
            .def_readonly("elbow", &franka::RobotState::elbow)
            .def_readonly("elbow_d", &franka::RobotState::elbow_d)
            .def_readonly("elbow_c", &franka::RobotState::elbow_c)
            .def_readonly("delbow_c", &franka::RobotState::delbow_c)
            .def_readonly("ddelbow_c", &franka::RobotState::ddelbow_c)
            .def_readonly("tau_J", &franka::RobotState::tau_J)
            .def_readonly("tau_J_d", &franka::RobotState::tau_J_d)
            .def_readonly("dtau_J", &franka::RobotState::dtau_J)
            .def_readonly("q", &franka::RobotState::q)
            .def_readonly("q_d", &franka::RobotState::q_d)
            .def_readonly("dq", &franka::RobotState::dq)
            .def_readonly("dq_d", &franka::RobotState::dq_d)
            .def_readonly("ddq_d", &franka::RobotState::ddq_d)
            .def_readonly("joint_contact", &franka::RobotState::joint_contact)
            .def_readonly("cartesian_contact", &franka::RobotState::cartesian_contact)
            .def_readonly("joint_collision", &franka::RobotState::joint_collision)
            .def_readonly("cartesian_collision", &franka::RobotState::cartesian_collision)
            .def_readonly("tau_ext_hat_filtered", &franka::RobotState::tau_ext_hat_filtered)
            .def_readonly("O_F_ext_hat_K", &franka::RobotState::O_F_ext_hat_K)
            .def_readonly("K_F_ext_hat_K", &franka::RobotState::K_F_ext_hat_K)
            .def_readonly("O_dP_EE_d", &franka::RobotState::O_dP_EE_d)
            .def_readonly("O_T_EE_c", &franka::RobotState::O_T_EE_c)
            .def_readonly("O_dP_EE_c", &franka::RobotState::O_dP_EE_c)
            .def_readonly("O_ddP_EE_c", &franka::RobotState::O_ddP_EE_c)
            .def_readonly("theta", &franka::RobotState::theta)
            .def_readonly("dtheta", &franka::RobotState::dtheta)
            .def_readonly("current_errors", &franka::RobotState::current_errors) //Errors
            .def_readonly("last_motion_errors", &franka::RobotState::last_motion_errors)       //Errors
            .def_readonly("control_command_success_rate", &franka::RobotState::control_command_success_rate)
            .def_readonly("robot_mode", &franka::RobotState::robot_mode)  //RobotMode
            .def_readonly("time", &franka::RobotState::time);  //Duration

    py::class_<franka::Finishable>(m, "Finishable")
            .def(py::init())
            .def_readwrite("motion_finished", &franka::Finishable::motion_finished);

    py::class_<franka::JointPositions, franka::Finishable>(m, "JointPositions")
            .def(py::init<const std::array<double, 7>&>())
            .def(py::init<std::initializer_list<double>>())
            .def_readwrite("q", &franka::JointPositions::q)
            .def_readwrite("motion_finished", &franka::JointPositions::motion_finished)
            ;

    py::class_<franka::CartesianPose, franka::Finishable>(m, "CartesianPose")
            .def(py::init<const std::array<double, 16>&>())
            .def(py::init<const std::array<double, 16>&, const std::array<double, 2>&>())
            .def(py::init<std::initializer_list<double>>())
            .def(py::init<std::initializer_list<double>, std::initializer_list<double>>())
            .def_readwrite("O_T_EE", &franka::CartesianPose::O_T_EE)
            .def_readwrite("elbow", &franka::CartesianPose::elbow)
            .def("hasElbow", &franka::CartesianPose::hasElbow)

            .def_readwrite("motion_finished", &franka::CartesianPose::motion_finished)
            ;

    py::class_<franka::Torques, franka::Finishable>(m, "Torques")
            .def(py::init<const std::array<double, 7>&>())
            .def(py::init<std::initializer_list<double>>())
            .def_readwrite("tau_J", &franka::Torques::tau_J)

            .def_readwrite("motion_finished", &franka::Torques::motion_finished)

            ;

    py::class_<franka::CartesianVelocities, franka::Finishable>(m, "CartesianVelocities")
            .def(py::init<const std::array<double, 6>&>())
            .def(py::init<const std::array<double, 6>&, const std::array<double, 2>&>())
            .def(py::init<std::initializer_list<double>>())
            .def(py::init<std::initializer_list<double>, std::initializer_list<double>>())

            .def_readwrite("O_dP_EE", &franka::CartesianVelocities::O_dP_EE)
            .def_readwrite("elbow", &franka::CartesianVelocities::elbow)

            .def("hasElbow", &franka::CartesianVelocities::hasElbow)

            .def_readwrite("motion_finished", &franka::CartesianVelocities::motion_finished)
            ;

    py::class_<franka::JointVelocities, franka::Finishable>(m, "JointVelocities")
            .def(py::init<const std::array<double, 7>&>())
            .def(py::init<std::initializer_list<double>>())
            .def_readwrite("dq", &franka::JointVelocities::dq)

            .def_readwrite("motion_finished", &franka::JointVelocities::motion_finished)

    ;

    py::class_<MotionGenerator>(m, "MotionGenerator")
            .def(py::init<double, const std::array<double, 7>>())
//            .def(py::self + py::self)  //franka::JointPositions
            .def("operator", &MotionGenerator::operator());
//            .def(franka::JointPositions() <const franka::RobotState&, franka::Duration>)
            ;
    m.def("JointPositionsMotionFinished", &JointPositionsMotionFinished);
    m.def("CartesianPoseMotionFinished", &CartesianPoseMotionFinished);
    m.def("TorquesMotionFinished", &TorquesMotionFinished);
    m.def("CartesianVelocitiesFinished", &CartesianVelocitiesFinished);
    m.def("JointVelocitiesFinished", &JointVelocitiesFinished);

    m.attr("kMaxCutoffFrequency") = franka::kMaxCutoffFrequency;
    m.attr("kDefaultCutoffFrequency") = franka::kDefaultCutoffFrequency;


    /**
     *  Gripper
     */

    py::class_<franka::GripperState>(m, "GripperState")
            .def(py::init())
            .def_readonly("width", &franka::GripperState::width)
            .def_readonly("max_width", &franka::GripperState::max_width)
            .def_readonly("is_grasped", &franka::GripperState::is_grasped)
            .def_readonly("temperature", &franka::GripperState::temperature)
            .def_readonly("time", &franka::GripperState::time)
            ;

    py::class_<franka::Gripper>(m, "Gripper")
            .def(py::init<const std::string&>())

            .def("homing", &franka::Gripper::homing)
            .def("grasp", &franka::Gripper::grasp)
            .def("move", &franka::Gripper::move)
            .def("stop", &franka::Gripper::stop)
            .def("readOnce", &franka::Gripper::readOnce)
            ;

//    m.def("test_path", &test_path);

}







