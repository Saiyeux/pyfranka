
from enum import IntEnum

class CommandEnum(IntEnum):

    NONE = 0
    IDLE = 1
    STOP = 2
    JOINT_SERVO = 3
    CARTESIAN_VELOCITY_EE = 4

