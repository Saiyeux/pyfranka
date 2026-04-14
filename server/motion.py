import numpy as np
from mathematics import skew_symmetric_matrix, linear_interpolation,\
    polynomial_interpolation, matrix2q
from pyfranka.franka_pybind import CartesianVelocities,\
    CartesianVelocitiesFinished, JointPositions, JointPositionsMotionFinished
import logging
from data_share import data_share_obj
from enum_data import CommandEnum
import time

# constant variables
DEFAULT_VELOCITY_ACC_FACTOR = 0.1
MAX_DP_EE = np.array([1.7, 1.7, 1.7, 2.5, 2.5, 2.5, 2.175])
MAX_DDP_EE = np.array([13, 13, 13, 25, 25, 25, 10])
MAX_DQ = np.array([2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5])
T_MIN = 200e-3


def set_franka_status(robot_state, data_obj):
    data_obj.TIMESTAMP.value = float(time.time())
    data_obj.O_T_EE[:] = matrix2q(robot_state.O_T_EE)
    data_obj.tau_J[:] = robot_state.tau_J
    data_obj.q[:] = robot_state.q
    data_obj.dq[:] = robot_state.dq
    data_obj.tau_ext_hat_filtered[:] = robot_state.tau_ext_hat_filtered
    data_obj.O_F_ext_hat_K[:] = robot_state.O_F_ext_hat_K
    data_obj.K_F_ext_hat_K[:] = robot_state.K_F_ext_hat_K
    data_obj.elbow[:] = robot_state.elbow


def velocity_transform_matrix(R, T):
    m = np.zeros((6, 6))
    m[:3, :3] = m[3:, 3:] = R
    m[:3, 3:] = skew_symmetric_matrix(T) @ R
    return m


def velocity_o2ee(velocity, tform):
    return velocity @ velocity_transform_matrix(tform[:3, :3].T, tform[3, :3])


def velocity_ee2o(velocity, tform):
    return velocity @ velocity_transform_matrix(tform[:3, :3], tform[3, :3])


class CartesianVeloctiyControlEE():
    def __init__(self):
        self.__time = 0.0
        self.__T = T_MIN
        self.__T_elbow = T_MIN
        self.__finished = False
        self.__v_start = np.zeros(6)
        self.__v_target = np.zeros(6)
        self.__v_ee = np.zeros(6)
        self.__elbow_start = np.zeros(2)
        self.__elbow_target = np.zeros(2)
        self.__delbow_start = np.zeros(2)
        self.__delbow_target = np.zeros(2)
        self.__ddelbow_start = np.zeros(2)
        self.__ddelbow_target = np.zeros(2)
        self.__elbow_d = np.zeros(2)
        self.__max_dp_EE = MAX_DP_EE * DEFAULT_VELOCITY_ACC_FACTOR
        self.__max_ddp_EE = MAX_DDP_EE * DEFAULT_VELOCITY_ACC_FACTOR

    def set_acc_factor(self, factor):
        self.__max_dp_EE = MAX_DP_EE * DEFAULT_VELOCITY_ACC_FACTOR
        self.__max_ddp_EE = MAX_DDP_EE * factor

    def calculate_interpolation_time(self, v0, v1):
        t = max(abs(v1 - v0) / self.__max_ddp_EE[:6])
        t = max(t, T_MIN)
        return t

    def pre_control(self, v):
        self.__v_target = np.array(v)

    def control(self, robot_state, period):
        self.__time += period.toSec()
        tcp = np.array(robot_state.O_T_EE_c).reshape(4, 4)
        set_franka_status(robot_state, data_share_obj)
        cmd = CommandEnum(data_share_obj.SET_FRANKA_COMMAND.value).name
        if cmd == 'NONE':
            return CartesianVelocities(np.zeros(6))
        elif cmd == 'IDLE':
            pass
        elif cmd == 'CARTESIAN_VELOCITY_EE':
            self.__v_target = np.array(
                data_share_obj.SET_REFERENCE_TCP_VELOCITY_EE[:6])
            self.__v_start = self.__v_ee
            self.set_acc_factor(
                data_share_obj.SET_CARTESIAN_VELOCITY_FACTOR.value)
            self.__time = period.toSec()
            self.__T = self.calculate_interpolation_time(
                self.__v_start, self.__v_target)
            self.__finished = False
            self.__elbow_start = robot_state.elbow_c
            self.__delbow_start = robot_state.delbow_c
            self.__ddelbow_start = robot_state.ddelbow_c
            self.__elbow_target = robot_state.elbow_c
            self.__elbow_target[
                0] = data_share_obj.SET_REFERENCE_TCP_VELOCITY_EE[6]
            self.__T_elbow = max(
                T_MIN,
                abs(self.__elbow_target[0] - self.__elbow_start[0]) /
                self.__max_dp_EE[6])
            data_share_obj.SET_FRANKA_COMMAND.value = CommandEnum.IDLE.value
            logging.debug(
                'Cartesian velocity EE is set to %s and elbow configure %s with acc factor %s'
                % (self.__v_target, self.__elbow_target,
                   data_share_obj.SET_CARTESIAN_VELOCITY_FACTOR.value))
        else:
            self.__v_target = np.zeros(6)
            self.__v_start = self.__v_ee
            self.__time = period.toSec()
            self.__T = self.calculate_interpolation_time(
                self.__v_start, self.__v_target)
            self.__finished = True
            self.__elbow_d = robot_state.elbow_c
            data_share_obj.SET_FRANKA_COMMAND.value = CommandEnum.IDLE.value
            logging.debug('Cartesian velocity EE Finished is set.')

        self.__v_ee = linear_interpolation(self.__time / self.__T,
                                           self.__v_start, self.__v_target)
        v_base = velocity_ee2o(self.__v_ee, tcp)
        self.__elbow_d = robot_state.elbow_c
        self.__elbow_d[0] = polynomial_interpolation(
            self.__time, self.__T_elbow, self.__elbow_start[0],
            self.__elbow_target[0], self.__delbow_start[0],
            self.__delbow_target[0], self.__ddelbow_start[0],
            self.__ddelbow_target[0])
        velocity = CartesianVelocities(v_base, self.__elbow_d)
        if self.__finished and self.__time > self.__T:
            logging.info('stop Cartesian Velocity control.')
            return CartesianVelocitiesFinished(velocity)
        return velocity


class JointServoControl():
    def __init__(self) -> None:

        self.__time = 0.0
        self.__T = T_MIN
        self.__finished = False
        self.__max_dq = MAX_DQ * DEFAULT_VELOCITY_ACC_FACTOR
        self.__q_d = np.zeros(7)
        self.__q_start = np.zeros(7)
        self.__q_target = np.zeros(7)
        self.__dq_start = np.zeros(7)
        self.__dq_target = np.zeros(7)
        self.__ddq_start = np.zeros(7)
        self.__ddq_target = np.zeros(7)

    def set_speed_factor(self, factor):
        self.__max_dq = MAX_DQ * factor

    def calculate_interpolation_time(self, q0, q1):
        t = max(abs(q1 - q0) / self.__max_dq)
        t = max(t, T_MIN)
        return t

    def pre_control(self, q):
        self.__q_d = np.array(q)

    def control(self, robot_state, period):
        self.__time += period.toSec()
        set_franka_status(robot_state, data_share_obj)
        cmd = CommandEnum(data_share_obj.SET_FRANKA_COMMAND.value).name
        if cmd in ['NONE', 'IDLE']:
            pass
        elif cmd == 'JOINT_SERVO':
            self.__q_start = self.__q_d
            self.__dq_start = np.array(robot_state.dq_d)
            self.__ddq_start = np.array(robot_state.ddq_d)
            self.__q_target = np.array(
                data_share_obj.SET_REFERENCE_JOINT_POSITION[:])
            self.set_speed_factor(data_share_obj.SET_JOINT_SERVO_FACTOR.value)
            self.__time = period.toSec()
            self.__T = self.calculate_interpolation_time(
                self.__q_start, self.__q_target)
            self.__finished = False
            data_share_obj.SET_FRANKA_COMMAND.value = CommandEnum.IDLE.value
            logging.debug('Joint servo to target %s' % self.__q_target)
        else:
            self.__finished = True
            data_share_obj.SET_FRANKA_COMMAND.value = CommandEnum.IDLE.value
            logging.debug('Joint servo Finished is set.')

        self.__q_d = polynomial_interpolation(self.__time, self.__T,
                                              self.__q_start, self.__q_target,
                                              self.__dq_start,
                                              self.__dq_target,
                                              self.__ddq_start,
                                              self.__ddq_target)
        position = JointPositions(self.__q_d)
        if self.__finished and self.__time > self.__T:
            logging.info('stop Joint Servo control.')
            return JointPositionsMotionFinished(position)
        return position
