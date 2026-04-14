
from pyfranka.franka_pybind import *
import time


def gripper_test():
    # franka_gripper_api_obj = FrankaGripperApi()
    # franka_gripper_api_obj.init_config("192.170.10.100")

    franka_gripper_api_obj = Gripper("192.170.10.100")

    grasping_width = 0.079  # m

    homing = True

    if homing:
        franka_gripper_api_obj.homing()

    # Check for the maximum grasping width.
    gripper_state = franka_gripper_api_obj.readOnce()
    print(gripper_state.max_width)
    if gripper_state.max_width < grasping_width:
        print("Object is too large for the current fingers on the gripper.")
        return -1

    # Grasp the object.
    if not franka_gripper_api_obj.grasp(grasping_width, 0.1, 60, 0.005, 0.005):
        print("Failed to grasp object.")
        return -1

    # Wait 3s and check afterwards, if the object is still grasped.
    time.sleep(3)

    gripper_state = franka_gripper_api_obj.readOnce()

    if not gripper_state.is_grasped:
        print("Object lost.")
        return -1

    print("Grasped object, will release it now.")
    franka_gripper_api_obj.stop()


try:
    gripper_test()
except Exception as e:
    print(Exception, e)



