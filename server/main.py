from server.mathematics import q2matrix
from gripper import GripperControl
from motion import CartesianVeloctiyControlEE, JointServoControl
from motion import set_franka_status
from rob_comm import RobComm
from pyfranka.franka_pybind import FrankaApi, RobotMode, MotionGenerator
import multiprocessing
import time
import sys
import logging
from data_share import data_share_obj
import os

logging.basicConfig(
    level=logging.DEBUG,
    format=
    '%(asctime)s [pid:%(process)d][thread:%(threadName)s %(thread)d] %(filename)s:%(funcName)s[line:%(lineno)d] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S')

FRANKA_IPADDR = '192.170.10.100'
REDIS_IPADDR = 'localhost'
REDIS_FRANKA_DB = 6
REDIS_GRIPPER_DB = 6
MAX_AUTO_RECOVERY_TIMES = 3


def update_robot_state(data_obj):
    logging.info('start the process of updating the robot state.')
    rob_comm_obj = RobComm(REDIS_IPADDR, REDIS_FRANKA_DB)
    while True:
        data_dic = {}
        data_dic["TIMESTAMP"] = data_obj.TIMESTAMP.value
        data_dic["O_T_EE"] = data_obj.O_T_EE[:]
        data_dic["tau_J"] = data_obj.tau_J[:]
        data_dic["q"] = data_obj.q[:]
        data_dic["dq"] = data_obj.dq[:]
        data_dic["tau_ext_hat_filtered"] = data_obj.tau_ext_hat_filtered[:]
        data_dic["O_F_ext_hat_K"] = data_obj.O_F_ext_hat_K[:]
        data_dic["K_F_ext_hat_K"] = data_obj.K_F_ext_hat_K[:]
        data_dic['elbow'] = data_obj.elbow[:]
        rob_comm_obj.write("GET_FRANKA_STATUS", data_dic)


def read_redis_data_loop(data_obj):
    logging.info('start the process of updating the robot state.')
    rob_comm_obj = RobComm(REDIS_IPADDR, REDIS_FRANKA_DB)
    while True:
        rob_comm_obj.get_franka_command(data_obj)
        time.sleep(0.001)


franka = FrankaApi()
franka.init_config(FRANKA_IPADDR, log_size=5000)
franka.set_default_behavior()

# Set the collision behavior.
lower_torque_thresholds_nominal = [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.]
upper_torque_thresholds_nominal = [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0]
lower_torque_thresholds_acceleration = [
    25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0
]
upper_torque_thresholds_acceleration = [
    35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0
]
lower_force_thresholds_nominal = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
upper_force_thresholds_nominal = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]
lower_force_thresholds_acceleration = [30.0, 30.0, 30.0, 25.0, 25.0, 25.0]
upper_force_thresholds_acceleration = [40.0, 40.0, 40.0, 35.0, 35.0, 35.0]

franka.set_collision_behavior(
    lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
    lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
    lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
    lower_force_thresholds_nominal, upper_force_thresholds_nominal)

Ready = False
for i in range(3):
    franka_state = franka.readOnce()
    if (franka_state.robot_mode == RobotMode.kIdle):
        Ready = True
    else:
        logging.error(
            'Robot is not ready to run, please check the robot or the enable button. Current Robot Mode is '
            + str(franka_state.robot_mode))
        if (franka_state.robot_mode in [RobotMode.kReflex]):
            logging.info(
                'Robot failed as the result of Reflex error, try to recovery.')
            franka.automatic_error_recovery()

if not Ready:
    sys.exit(1)


def franka_robot_control_loop():
    rc = RobComm(REDIS_IPADDR, REDIS_FRANKA_DB)
    gc = GripperControl(FRANKA_IPADDR, REDIS_IPADDR, REDIS_GRIPPER_DB)

    cv = CartesianVeloctiyControlEE()
    js = JointServoControl()

    rc.set_stopped_status(True)

    rc.clear_command()
    set_franka_status(franka.readOnce(), data_share_obj)

    auto_recovery_counter = 0
    result = True

    logging.info('start the franka control state machine.')
    while True:
        rc.get_franka_command(data_share_obj)
        gc.control()
        if rc.cmd in ['NONE', 'IDLE', 'STOP']:
            continue
        elif rc.cmd == 'JOINT_SERVO':
            rc.set_stopped_status(False)
            logging.info('start Joint Servo control')
            js.pre_control(franka.readOnce().q_d)
            result = franka.robot_control(joint_positions_handle=js.control)
            rc.set_stopped_status(True)
        elif rc.cmd == 'CARTESIAN_VELOCITY_EE':
            rc.set_stopped_status(False)
            logging.info(
                'start Cartesian Veloctiy contrl on End Effector frame')
            result = franka.robot_control(
                cartesian_velocities_handle=cv.control)
            rc.set_stopped_status(True)
        elif rc.cmd == 'JOINT_P2P':
            rc.set_stopped_status(False)
            rc.clear_command()
            target = data_share_obj.SET_REFERENCE_JOINT_POSITION[:]
            factor = data_share_obj.SET_JOINT_SERVO_FACTOR.value
            mg = MotionGenerator(factor, target)
            logging.info(
                'start Joint point to point cotrol to target %s with speed factor %f'
                % (target, factor))
            result = franka.robot_control(joint_positions_handle=mg.operator)
            set_franka_status(franka.readOnce(), data_share_obj)
            rc.set_stopped_status(True)
        elif rc.cmd == 'SET_END_EFFECTOR':
            rc.clear_command()
            ee = data_share_obj.SET_END_EFFECTOR.value
            franka.setEE(q2matrix(ee))
            logging.info('set end effector to %s as matrix is %s' %
                         (ee, q2matrix(ee)))
        else:
            logging.error('Unsupported command %s recieved.' % rc.cmd)

        if not result:
            auto_recovery_counter += 1
            if auto_recovery_counter >= MAX_AUTO_RECOVERY_TIMES:
                logging.info(
                    'Max auto recovery is exceeded %s times, and quit the franka service'
                    % MAX_AUTO_RECOVERY_TIMES)
                return
        else:
            auto_recovery_counter = 0


write_redis_data_p = multiprocessing.Process(target=update_robot_state,
                                             name='update_robot_state',
                                             args=(data_share_obj, ),
                                             daemon=True)

read_redis_data_p = multiprocessing.Process(target=read_redis_data_loop,
                                            name='read_redis_data_loop',
                                            args=(data_share_obj, ),
                                            daemon=True)

franka_robot_control_p = multiprocessing.Process(
    target=franka_robot_control_loop,
    name='franka_robot_control_loop',
    daemon=True)

# set EE to Gripper finger
franka.setEE(
    [0.7071, -0.7071, 0, 0, 0.7071, 0.7071, 0, 0, 0, 0, 1, 0, 0, 0, 0.1034, 1])

write_redis_data_p.start()
read_redis_data_p.start()
franka_robot_control_p.start()

# write_redis_data_p.join()
# read_redis_data_p.join()
franka_robot_control_p.join()
