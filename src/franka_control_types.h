//
// Created by cyshi on 2021/1/13.
//
#pragma once
#include <franka/control_types.h>

inline franka::JointPositions JointPositionsMotionFinished(  // NOLINT(readability-identifier-naming)
        franka::JointPositions command) noexcept {
    command.motion_finished = true;
    return command;
}

inline franka::CartesianPose CartesianPoseMotionFinished(  // NOLINT(readability-identifier-naming)
        franka::CartesianPose command) noexcept {
    command.motion_finished = true;
    return command;
}

inline franka::Torques TorquesMotionFinished(
        franka::Torques command) noexcept {  // NOLINT(readability-identifier-naming)
    command.motion_finished = true;
    return command;
}

inline franka::CartesianVelocities CartesianVelocitiesFinished(
        franka::CartesianVelocities command) noexcept {  // NOLINT(readability-identifier-naming)
    command.motion_finished = true;
    return command;
}

inline franka::JointVelocities JointVelocitiesFinished(
        franka::JointVelocities command) noexcept {  // NOLINT(readability-identifier-naming)
command.motion_finished = true;
return command;
}


