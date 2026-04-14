
from pyfranka.franka_pybind import *
import numpy as np

# connect to robot
franka_api_obj = FrankaApi()
franka_api_obj.init_config("192.170.10.100")

franka_api_obj.automatic_error_recovery()

franka_api_obj.set_default_behavior()
# load the kinematics and dynamics model
model = franka_api_obj.loadModel()

initial_state = franka_api_obj.readOnce()

# Bias torque sensor
mass_array = model.mass(initial_state)

print(mass_array)





