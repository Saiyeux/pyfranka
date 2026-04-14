from functools import wraps
import multiprocessing
import time
from ctypes import c_char_p

def Singleton(cls):
    instances = {}
    @wraps(cls)
    def getinstance(*args, **kw):
        if cls not in instances:
            instances[cls] = cls(*args, **kw)
        return instances[cls]
    return getinstance

@Singleton
class DataShare(object):
    def __init__(self):

        self.lock = multiprocessing.Manager().Lock()

        ### data ###
        self.TIMESTAMP = multiprocessing.Value('d', 0.0)
        self.O_T_EE = multiprocessing.Array('d', [0]*7)
        self.tau_J = multiprocessing.Array('d', [0]*7)
        self.q = multiprocessing.Array('d', [0]*7)
        self.dq = multiprocessing.Array('d', [0]*7)
        self.tau_ext_hat_filtered = multiprocessing.Array('d', [0]*7)
        self.O_F_ext_hat_K = multiprocessing.Array('d', [0]*6)
        ### data ###
        self.error_count = multiprocessing.Value('i', 0)

        self.robot_mode = multiprocessing.Manager().Value(c_char_p, "ss")

    def get_lock(self):
        return self.lock

data_share_obj = DataShare()

if __name__ == '__main__':
    pass