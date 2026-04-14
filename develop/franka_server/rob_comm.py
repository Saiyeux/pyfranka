import redis
import msgpack
import time
from server.mathematics import matrix2q
import logging
from enum_data import CommandEnum


class RedisComm():
    def __init__(self, host, database):
        self.__r = redis.StrictRedis(host=host, port=6379, db=database)

    def write(self, key, value):
        try:
            self.__r.set(key.encode('utf-8'), msgpack.dumps(value))
        except Exception as e:
            print(Exception, e)
            return None

    def read(self, key):
        try:
            return msgpack.unpackb(self.__r.get(key.encode('utf-8')))
        except Exception as e:
            print(Exception, e)
            print('data is not existed')
            return None


class GripperComm(RedisComm):
    def __init__(self, host, database):
        super().__init__(host, database)
        self.cmd = 'None'
        self.width = 0.
        self.speed = 0.
        self.force = 0.
        self.epsilon_inner = 5e-3
        self.epsilon_outer = 5e-3

    def clear_command(self):
        cmd = {}
        cmd['COMMAND'] = 'IDLE'
        self.write('SET_GRIPPER_COMMAND', cmd)

    def set_gripper_status(self, gripper_state):
        state = {}
        state['TIMESTAMP'] = time.time()
        state['WIDTH'] = gripper_state.width
        state['IS_GRASPED'] = gripper_state.is_grasped
        state['TEMPERATURE'] = gripper_state.temperature
        state['MAX_WIDTH'] = gripper_state.max_width
        self.write('GET_GRIPPER_STATUS', state)

    def get_gripper_command(self):
        cmd = self.read('SET_GRIPPER_COMMAND')
        self.clear_command()
        self.cmd = cmd['COMMAND']
        if self.cmd == 'NONE' or self.cmd == 'IDLE' or self.cmd == 'STOP' or \
                self.cmd == 'HOMING':
            pass
        elif self.cmd == 'GRASP':
            self.width = cmd['WIDTH']
            self.speed = cmd['SPEED']
            self.force = cmd['FORCE']
            self.epsilon_inner = cmd['EPSILON_INNER']
            self.epsilon_outer = cmd['EPSILON_OUTER']
        elif self.cmd == 'MOVE':
            self.width = cmd['WIDTH']
            self.speed = cmd['SPEED']
        else:
            logging.error('Unsupported command %s recieved.' % self.cmd)


class RobComm(RedisComm):
    def __init__(self, host, database):
        super().__init__(host, database)
        self.cmd = 'NONE'

    def clear_command(self):
        cmd = self.read('SET_FRANKA_COMMAND')
        if cmd['COMMAND'] != 'IDLE':
            cmd['COMMAND'] = 'IDLE'
            self.write('SET_FRANKA_COMMAND', cmd)

    def get_franka_command(self, data_share_obj):

        cmd = self.read('SET_FRANKA_COMMAND')
        if not cmd['COMMAND']:
            return
        else:
            self.cmd = cmd['COMMAND']
        if self.cmd == 'NONE':
            data_share_obj.SET_FRANKA_COMMAND.value = CommandEnum.NONE.value
        elif self.cmd == 'IDLE':
            data_share_obj.SET_FRANKA_COMMAND.value = CommandEnum.IDLE.value
        elif self.cmd == 'STOP':
            data_share_obj.SET_FRANKA_COMMAND.value = CommandEnum.STOP.value
        elif self.cmd == 'JOINT_SERVO':
            data_share_obj.SET_FRANKA_COMMAND.value = CommandEnum.JOINT_SERVO.value
            data_share_obj.SET_REFERENCE_JOINT_POSITION[:] = cmd['SET_REFERENCE_JOINT_POSITION']
            # print("get_franka_command SET_REFERENCE_JOINT_POSITION", data_share_obj.SET_REFERENCE_JOINT_POSITION[:])
        elif self.cmd == 'CARTESIAN_VELOCITY_EE':
            data_share_obj.SET_FRANKA_COMMAND.value = CommandEnum.CARTESIAN_VELOCITY_EE.value
            data_share_obj.SET_REFERENCE_TCP_VELOCITY_EE[:] = cmd['SET_REFERENCE_TCP_VELOCITY_EE']
        else:
            logging.error('Unsupported command %s recieved.' % self.cmd)

    def set_franka_status(self, robot_state):
        state = {}
        state['TIMESTAMP'] = time.time()
        state['O_T_EE'] = matrix2q(robot_state.O_T_EE)
        # state['O_T_EE_d'] = robot_state.O_T_EE_d
        # state['F_T_EE'] = robot_state.F_T_EE
        # state['F_T_NE'] = robot_state.F_T_NE
        # state['NE_T_EE'] = robot_state.NE_T_EE
        # state['EE_T_K'] = robot_state.EE_T_K
        # state['m_ee'] = robot_state.m_ee
        # state['I_ee'] = robot_state.I_ee
        # state['F_x_Cee'] = robot_state.F_x_Cee
        # state['m_load'] = robot_state.m_load
        # state['I_load'] = robot_state.I_load
        # state['F_x_Cload'] = robot_state.F_x_Cload
        # state['m_total'] = robot_state.m_total
        # state['I_total'] = robot_state.I_total
        # state['F_x_Ctotal'] = robot_state.F_x_Ctotal
        # state['elbow'] = robot_state.elbow
        # state['elbow_d'] = robot_state.elbow_d
        # state['elbow_c'] = robot_state.elbow_c
        # state['delbow_c'] = robot_state.delbow_c
        # state['ddelbow_c'] = robot_state.ddelbow_c
        state['tau_J'] = robot_state.tau_J
        # state['tau_J_d'] = robot_state.tau_J_d
        # state['dtau_J'] = robot_state.dtau_J
        state['q'] = robot_state.q
        # state['q_d'] = robot_state.q_d
        state['dq'] = robot_state.dq
        # state['dq_d'] = robot_state.dq_d
        # state['ddq_d'] = robot_state.ddq_d
        # state['joint_contact'] = robot_state.joint_contact
        # state['cartesian_contact'] = robot_state.cartesian_contact
        # state['joint_collision'] = robot_state.joint_collision
        # state['cartesian_collision'] = robot_state.cartesian_collision
        state['tau_ext_hat_filtered'] = robot_state.tau_ext_hat_filtered
        state['O_F_ext_hat_K'] = robot_state.O_F_ext_hat_K
        # state['K_F_ext_hat_K'] = robot_state.K_F_ext_hat_K
        # state['O_dP_EE_d'] = robot_state.O_dP_EE_d
        # state['O_T_EE_c'] = robot_state.O_T_EE_c
        # state['O_dP_EE_c'] = robot_state.O_dP_EE_c
        # state['O_ddP_EE_c'] = robot_state.O_ddP_EE_c
        # state['theta'] = robot_state.theta
        # state['dtheta'] = robot_state.dtheta
        # state['current_errors'] = str(robot_state.current_errors)
        # state['last_motion_errors'] = robot_state.last_motion_errors
        # state[
        #     'control_command_success_rate'] = robot_state.control_command_success_rate
        # state['robot_mode'] = str(robot_state.robot_mode)
        # state['time'] = robot_state.time.toSec()
        self.write('GET_FRANKA_STATUS', state)
