#include <cassert>
#include <cmath>
#include <iostream>

#include "../src/j7_pinned_solver.h"

using j7_pinned::Matrix67d;
using j7_pinned::Vector6d;
using j7_pinned::Vector7d;

int main() {
    Matrix67d J;
    J << -0.147485943, -0.01896477, -0.1304477, 0.2354025397, 0.0207756908, 0.0928566272, 0,
          0.4304452799, 0.0502932481, 0.2501241601, 0.3958420912, 0.0445181234, -0.046824459, 0,
          0, -0.0138745676, -0.3583390711, 0.2354359426, 0.0016975436, 0.0915320535, 0,
          -1e-10, 0.9356865867, -0.2779984283, 0.1949475195, 0.8081534448, -0.4226424924, 0.0454011248,
          0, 0.352832271, 0.7372324526, -0.5844246038, -0.3593549008, -0.9056383392, 0.0168984992,
          1, 1e-10, 0.6157963824, 0.787682263, -0.4666391161, -0.0345329183, -0.9988259;

    const Eigen::Vector3d tool_axis(0.0454011248, 0.0168984992, -0.9988259);
    Vector7d q;
    q << -1.210, -0.907, 1.547, -2.461, 1.138, 2.091, -0.021;
    Vector7d q_ref = q;

    Vector6d v = Vector6d::Zero();
    v << 0.015, -0.006, -0.010, 0.020, -0.015, 0.030;

    const Vector7d dq = j7_pinned::solve_velocity(
        J, v, tool_axis, q, q_ref, -0.021, 3.0, 0.3, 0.02);

    assert(std::abs(dq(6)) < 1e-6);

    Eigen::Vector3d u;
    Eigen::Vector3d w;
    j7_pinned::perpendicular_basis(tool_axis.normalized(), &u, &w);
    Matrix67d M = Matrix67d::Zero();
    M.block<3, 7>(0, 0) = J.block<3, 7>(0, 0);
    M.row(3) = u.transpose() * J.block<3, 7>(3, 0);
    M.row(4) = w.transpose() * J.block<3, 7>(3, 0);
    M(5, 6) = 1.0;

    Vector6d b = Vector6d::Zero();
    b.head<3>() = v.head<3>();
    b(3) = u.dot(v.tail<3>());
    b(4) = w.dot(v.tail<3>());
    b(5) = 0.0;

    const double residual = (M * dq - b).norm();
    assert(residual < 5e-3);

    q(6) = 0.079;
    const Vector7d correction = j7_pinned::solve_velocity(
        J, Vector6d::Zero(), tool_axis, q, q_ref, -0.021, 3.0, 0.3, 0.02);
    assert(correction(6) < -0.05);

    std::cout << "j7 pinned solver regression passed\n";
    return 0;
}
