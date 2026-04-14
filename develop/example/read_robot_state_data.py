

from pyfranka.franka_pybind import *

def read_data(robot_state):

    print(robot_state.q)

    return False


franka_api_obj = FrankaApi()

franka_api_obj.init_config("192.170.10.100")

franka_api_obj.read_robot_state(read_data)
