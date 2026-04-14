from server.cartesian_motion import CartesianVeloctiyControlEE
import multiprocessing
from server.rob_comm import RedisComm
from pyfranka.franka_pybind import *

FRANKA_IPADDR = '192.170.10.100'
REDIS_IPADDR = 'localhost'
REDIS_DB = 6

# rc = RedisComm(REDIS_IPADDR, REDIS_DB)
# shared_data = multiprocessing.Manager().dict()
# p = multiprocessing.Process(target=rc.update_redis, args=(shared_data, ))
# p.start()

franka = FrankaApi()
franka.init_config(FRANKA_IPADDR)
franka.set_default_behavior()
# franka_state = franka.readOnce()

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

# cv = CartesianVeloctiyControlEE(0.1)
# cv.set_reference_callback(rc.get_reference_tcp_velocity_ee)
# franka.robot_control(cartesian_velocity_handle=cv.control)


class Test():
    def __init__(self) -> None:
        pass

    def control(self, robot_state, period):
        print(robot_state.q)
        obj = CartesianVelocities(robot_state.O_dP_EE_c)
        return obj


class Test1():
    def __init__(self) -> None:
        pass

    def read(self, robot_state):
        print(robot_state.q)
        return False


t = Test()
t1 = Test1()
franka.robot_control(cartesian_velocity_handle=t.control)
# franka.read_robot_state(t.read)
