

import redis
import msgpack
from enum import Enum
import time
import math


class RedisBase():
    def __init__(self, host, database, socket_connect_timeout):
        self.__r = redis.StrictRedis(host=host, port=6379, db=database, socket_connect_timeout=socket_connect_timeout)

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
            print("data is not existed")
            return None


class ControlState(Enum):
 
    POSITION_CONTROL_1_STATE = 1 
    POSITION_CONTROL_2_STATE = 2 
    POSITION_CONTROL_3_STATE = 3 


def run_state_machine():

    control_state = ControlState.POSITION_CONTROL_1_STATE
    redis_obj = RedisBase("localhost", 6, 1)

    target_point_1 = [-0.13511041814986435, -0.34952914405287355, -0.24549207995206182, -1.980354938370613, -0.12262119028082599, 1.677436942153507, 0.28921527461735264]
    target_point_2 = [0.10288658271024102, -0.4965784313636747, 0.3964953119085546, -2.0248876549904806, 0.14466717973020338, 1.6380571499402652, 1.297923556195365]
    target_point_3 = [-0.03650818808507501, -0.290549163441909, 0.33733649142996497, -2.5904481121531706, 0.14166160340110462, 2.399622288360946, 0.9705992063151465]

    while True:
        
        redis_obj.write("CURRENT_CONTROL_STATE", control_state.name)

        current_point = redis_obj.read("GET_REALTIME_JOINT_POSITION")

        if control_state == ControlState.POSITION_CONTROL_1_STATE:
            error_list = list(map(lambda x, y: abs(x-y), current_point, target_point_1))
            result_list = list(map(lambda x: x*1000 < 1, error_list))

            result = False
            if not False in result_list:
                result = True
            
            if result == True:
                control_state = ControlState.POSITION_CONTROL_2_STATE
            else:
                control_state = ControlState.POSITION_CONTROL_1_STATE
                redis_obj.write("SET_REFERENCE_JOINT_POSITION", target_point_1)
            
        elif control_state == ControlState.POSITION_CONTROL_2_STATE:
            error_list = list(map(lambda x, y: abs(x-y), current_point, target_point_2))
            result_list = list(map(lambda x: x*1000 < 1, error_list))

            result = False
            if not False in result_list:
                result = True

            if result == True:
                control_state = ControlState.POSITION_CONTROL_3_STATE
            else:
                control_state = ControlState.POSITION_CONTROL_2_STATE
                redis_obj.write("SET_REFERENCE_JOINT_POSITION", target_point_2)

        elif control_state == ControlState.POSITION_CONTROL_3_STATE:
            error_list = list(map(lambda x, y: abs(x-y), current_point, target_point_3))
            result_list = list(map(lambda x: x*1000 < 1, error_list))

            result = False
            if not False in result_list:
                result = True

            if result == True:
                control_state = ControlState.POSITION_CONTROL_1_STATE
            else:
                control_state = ControlState.POSITION_CONTROL_3_STATE
                redis_obj.write("SET_REFERENCE_JOINT_POSITION", target_point_3)


if __name__ == "__main__":
    run_state_machine()
    