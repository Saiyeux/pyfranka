from math import log
from gripper import GripperControl
from motion import CartesianVeloctiyControlEE, JointServoControl
from motion import set_franka_status
from rob_comm import RobComm
from pyfranka.franka_pybind import FrankaApi
import multiprocessing
import time
import logging
from data_share import data_share_obj

logging.basicConfig(
    level=logging.INFO,
    format=
    '%(asctime)s [pid:%(process)d][thread:%(threadName)s %(thread)d] %(filename)s:%(funcName)s[line:%(lineno)d] %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S')

FRANKA_IPADDR = '192.170.10.100'
REDIS_IPADDR = 'localhost'
REDIS_FRANKA_DB = 6
REDIS_GRIPPER_DB = 6
REDIS_UPDATED_INTERVAL = 1e-3


def update_robot_state(data_obj):
    logging.info('start the process of updating the robot state.')

    rob_comm_obj = RobComm(REDIS_IPADDR, REDIS_FRANKA_DB)

    while True:
        t1 = time.time()

        data_dic = {}
        data_dic["TIMESTAMP"] = data_obj.TIMESTAMP.value
        data_dic["O_T_EE"] = data_obj.O_T_EE[:]
        data_dic["tau_J"] = data_obj.tau_J[:]
        data_dic["q"] = data_obj.q[:]
        data_dic["dq"] = data_obj.dq[:]
        data_dic["tau_ext_hat_filtered"] = data_obj.tau_ext_hat_filtered[:]
        data_dic["O_F_ext_hat_K"] = data_obj.O_F_ext_hat_K[:]
       
        rob_comm_obj.write("GET_FRANKA_STATUS", data_dic) 

        # rob_comm_obj.write("TIMESTAMP", data_obj.TIMESTAMP.value) 
        # rob_comm_obj.write("O_T_EE", data_obj.O_T_EE[:])
        # rob_comm_obj.write("tau_J", data_obj.tau_J[:])
        # rob_comm_obj.write("q", data_obj.q[:])
        # rob_comm_obj.write("dq", data_obj.dq[:])
        # rob_comm_obj.write("tau_ext_hat_filtered", data_obj.tau_ext_hat_filtered[:])
        # rob_comm_obj.write("O_F_ext_hat_K", data_obj.O_F_ext_hat_K[:])

        t2 = time.time()
        if (t2 - t1) < 0.001:
            time.sleep(0.001 - float(t2 - t1))
        # print("update_robot_state: ", t2 - t1)

def read_redis_data_loop(data_obj):
    logging.info('start the process of updating the robot state.')
    rob_comm_obj = RobComm(REDIS_IPADDR, REDIS_FRANKA_DB)
    while True:
        rob_comm_obj.get_franka_command(data_obj)
        # rob_comm_obj.clear_command()
        time.sleep(0.001)


franka = FrankaApi()
franka.init_config(FRANKA_IPADDR)
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

def franka_robot_control_loop():
    rc = RobComm(REDIS_IPADDR, REDIS_FRANKA_DB)

    cv = CartesianVeloctiyControlEE()
    js = JointServoControl()

    rc.clear_command()
    set_franka_status(franka.readOnce(), data_share_obj)

    logging.info('start the franka control state machine.')
    while True:
        rc.get_franka_command(data_share_obj)
        if rc.cmd in ['NONE', 'IDLE', 'STOP']:
            continue
        elif rc.cmd == 'JOINT_SERVO':
            logging.info('start Joint Servo control')
            js.pre_control(franka.readOnce().q_d)
            franka.robot_control(joint_positions_handle=js.control)
        elif rc.cmd == 'CARTESIAN_VELOCITY_EE':
            logging.info('start Cartesian Veloctiy contrl on End Effector frame')
            franka.robot_control(cartesian_velocities_handle=cv.control)
        else:
            logging.error('Unsupported command %s recieved.' % rc.cmd)


write_redis_data_p = multiprocessing.Process(target=update_robot_state,
                             name='update_robot_state',
                             args=(data_share_obj,),
                             daemon=True)

read_redis_data_p = multiprocessing.Process(target=read_redis_data_loop,
                                            name='read_redis_data_loop',
                                            args=(data_share_obj,),
                                            daemon=True)

franka_robot_control_p = multiprocessing.Process(target=franka_robot_control_loop,
                                                 name='franka_robot_control_loop',
                                                 daemon=True)

# gc = GripperControl(FRANKA_IPADDR, REDIS_IPADDR, REDIS_GRIPPER_DB)
# p2 = multiprocessing.Process(target=gc.control,
#                              name='gripper_control',
#                              daemon=True)

write_redis_data_p.start()
read_redis_data_p.start()
franka_robot_control_p.start()
# p2.start()

write_redis_data_p.join()
read_redis_data_p.join()
franka_robot_control_p.join()
