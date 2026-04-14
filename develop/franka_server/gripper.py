from pyfranka.franka_pybind import Gripper
from rob_comm import GripperComm
import logging


class GripperControl():
    def __init__(self, franka_ip, redis_ip, db) -> None:
        self.__gripper = Gripper(franka_ip)
        self.__gc = GripperComm(redis_ip, db)

    def control(self):
        logging.info('start the gripper control process.')
        while True:
            self.__gc.set_gripper_status(self.__gripper.readOnce())
            self.__gc.get_gripper_command()
            if self.__gc.cmd in ['NONE', 'IDLE']:
                pass
            elif self.__gc.cmd == 'HOMING':
                self.__gripper.homing()
            elif self.__gc.cmd == 'GRASP':
                self.__gripper.grasp(self.__gc.width, self.__gc.speed,
                                     self.__gc.force, self.__gc.epsilon_inner,
                                     self.__gc.epsilon_outer)

            elif self.__gc.cmd == 'MOVE':
                self.__gripper.move(self.__gc.width, self.__gc.speed)

            elif self.__gc.cmd == 'STOP':
                self.__gripper.stop()
            else:
                logging.error('Unsupported commamd %s recieved.' %
                              self.__gc.cmd)
