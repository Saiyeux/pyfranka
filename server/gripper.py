import redis
from pyfranka.franka_pybind import Gripper
from rob_comm import GripperComm
import logging

GRIPPER_STATUS_UPDATE_INTERVAL = 50e-3


class GripperControl(GripperComm):
    def __init__(self, franka_ip, redis_ip, db) -> None:
        super().__init__(redis_ip, db)
        self.__gripper = Gripper(franka_ip)

    def control(self):
        self.get_gripper_command()
        if self.cmd in ['NONE', 'IDLE']:
            pass
        elif self.cmd == 'HOMING':
            self.clear_command()
            logging.info('set Gripper HOMING')
            self.__gripper.homing()
            logging.info('finish Gripper Command HOMING')
        elif self.cmd == 'GRASP':
            self.clear_command()
            logging.info(
                'set Gripper GRASP with parameter: width %f(inner epsilon %f, outer epsilon %f) speed %f, force %f'
                % (self.width, self.speed, self.force, self.epsilon_inner,
                   self.epsilon_outer))
            self.__gripper.grasp(self.width, self.speed, self.force,
                                 self.epsilon_inner, self.epsilon_outer)
            logging.info('finish Gripper Command GRASP')
        elif self.cmd == 'MOVE':
            self.clear_command()
            logging.info(
                'set Gripper MOVE with parameter: width %f, speed %f' %
                (self.width, self.speed))
            self.__gripper.move(self.width, self.speed)
            logging.info('finish Gripper Command MOVE')
        elif self.cmd == 'STOP':
            self.clear_command()
            logging.info('set Gripper STOP')
            self.__gripper.stop()
            logging.info('finish Gripper Command STOP')
        else:
            logging.error('Unsupported commamd %s recieved.' % self.cmd)
