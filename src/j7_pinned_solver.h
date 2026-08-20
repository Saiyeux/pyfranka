#pragma once

#include <array>
#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>

namespace j7_pinned {

using Matrix67d = Eigen::Matrix<double, 6, 7>;
using Matrix76d = Eigen::Matrix<double, 7, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

inline Eigen::Vector3d normalized_or_z(const Eigen::Vector3d& v) {
    const double n = v.norm();
    if (n < 1e-9) {
        return Eigen::Vector3d::UnitZ();
    }
    return v / n;
}

inline void perpendicular_basis(const Eigen::Vector3d& axis,
                                Eigen::Vector3d* u,
                                Eigen::Vector3d* w) {
    Eigen::Vector3d helper = Eigen::Vector3d::UnitZ();
    if (std::abs(axis.dot(helper)) > 0.9) {
        helper = Eigen::Vector3d::UnitX();
    }
    *u = normalized_or_z(helper.cross(axis));
    *w = normalized_or_z(axis.cross(*u));
}

inline Vector7d solve_velocity(const Matrix67d& J,
                               const Vector6d& cartesian_velocity,
                               const Eigen::Vector3d& tool_axis,
                               const Vector7d& q,
                               const Vector7d& q_reference,
                               double q7_target,
                               double j7_gain,
                               double posture_gain,
                               double damping) {
    const Eigen::Vector3d axis = normalized_or_z(tool_axis);
    Eigen::Vector3d u;
    Eigen::Vector3d w;
    perpendicular_basis(axis, &u, &w);

    Matrix67d M = Matrix67d::Zero();
    M.block<3, 7>(0, 0) = J.block<3, 7>(0, 0);
    M.row(3) = u.transpose() * J.block<3, 7>(3, 0);
    M.row(4) = w.transpose() * J.block<3, 7>(3, 0);
    M(5, 6) = 1.0;

    Vector6d b = Vector6d::Zero();
    b.head<3>() = cartesian_velocity.head<3>();
    b(3) = u.dot(cartesian_velocity.tail<3>());
    b(4) = w.dot(cartesian_velocity.tail<3>());
    b(5) = j7_gain * (q7_target - q(6));

    const double lam2 = damping * damping;
    Matrix76d M_pinv =
        M.transpose() *
        (M * M.transpose() + lam2 * Eigen::Matrix<double, 6, 6>::Identity()).inverse();

    Vector7d dq = M_pinv * b;

    Vector7d q_ref = q_reference;
    q_ref(6) = q7_target;
    const Eigen::Matrix<double, 7, 7> N =
        Eigen::Matrix<double, 7, 7>::Identity() - M_pinv * M;
    dq += N * (posture_gain * (q_ref - q));
    return dq;
}

inline double min_joint_margin(const std::array<double, 7>& q,
                               int* joint_index = nullptr) {
    static const std::array<double, 7> q_min = {
        -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
    static const std::array<double, 7> q_max = {
         2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};

    double best = 1e9;
    int best_i = 0;
    for (int i = 0; i < 7; ++i) {
        const double margin = std::min(q[i] - q_min[i], q_max[i] - q[i]);
        if (margin < best) {
            best = margin;
            best_i = i;
        }
    }
    if (joint_index != nullptr) {
        *joint_index = best_i;
    }
    return best;
}

}  // namespace j7_pinned
