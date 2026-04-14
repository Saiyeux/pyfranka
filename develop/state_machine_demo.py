
from enum import Enum


class RobotControlState(Enum):

    GRASP_THE_TEST_TUBE_STATE = 1  # 抓试管
    OPEN_LOOP_ROUGH_ALIGNMENT_STATE = 2 # 开环粗对准
    THE_CLOSED_LOOP_STATE = 3 # 闭环
    STABLE_MOTIONLESS_STATE = 4 # 稳定不动
    FORCE_CONTROL_INSERT_STATE = 5 # 力控插入
    ROBOT_TASK_FINISH_STATE = 6  # 机械臂任务完成


def test():

    robot_control_state = RobotControlState.GRASP_THE_TEST_TUBE_STATE
    while True:

        if robot_control_state == RobotControlState.GRASP_THE_TEST_TUBE_STATE:  # 抓试管
            # 先执行对应状态下的功能函数，然后根据函数返回的结果来判断进入哪个状态
            if True: # 抓试管成功
                print("抓试管成功")
                robot_control_state = RobotControlState.OPEN_LOOP_ROUGH_ALIGNMENT_STATE
            else:
                print("抓试管失败")
                robot_control_state = RobotControlState.GRASP_THE_TEST_TUBE_STATE

        elif robot_control_state == RobotControlState.OPEN_LOOP_ROUGH_ALIGNMENT_STATE:  # 开环粗对准
            # 先执行对应状态下的功能函数，然后根据函数返回的结果来判断进入哪个状态
            if True: # 开环粗对准成功
                print("开环粗对准成功")
                robot_control_state = RobotControlState.THE_CLOSED_LOOP_STATE
            else:
                print("开环粗对准失败")
                robot_control_state = RobotControlState.OPEN_LOOP_ROUGH_ALIGNMENT_STATE

        elif robot_control_state == RobotControlState.THE_CLOSED_LOOP_STATE:    # 闭环
            # 先执行对应状态下的功能函数，然后根据函数返回的结果来判断进入哪个状态
            if True:  # 误差足够
                print("闭环:　误差足够")
                robot_control_state = RobotControlState.STABLE_MOTIONLESS_STATE
            else:
                print("闭环:　误差不够")
                robot_control_state = RobotControlState.THE_CLOSED_LOOP_STATE

        elif robot_control_state == RobotControlState.STABLE_MOTIONLESS_STATE:  # 稳定不动
            # 先执行对应状态下的功能函数，然后根据函数返回的结果来判断进入哪个状态
            state = 3
            if state == 1:  # 如果目标不动
                print("稳定不动:　目标不动")
                robot_control_state = RobotControlState.STABLE_MOTIONLESS_STATE
            elif state == 2: # 若目标动了
                print("稳定不动:　目标动了")
                robot_control_state = RobotControlState.THE_CLOSED_LOOP_STATE
            elif state == 3: # 如果准备好插入
                print("稳定不动:　准备好插入")
                robot_control_state = RobotControlState.FORCE_CONTROL_INSERT_STATE
            else:
                pass

        elif robot_control_state == RobotControlState.FORCE_CONTROL_INSERT_STATE:   # 力控插入
            # 先执行对应状态下的功能函数，然后根据函数返回的结果来判断进入哪个状态
            if True: # 力控插入成功
                print("力控插入成功")
                robot_control_state = RobotControlState.ROBOT_TASK_FINISH_STATE
            else:
                print(力控插入失败)
                robot_control_state = RobotControlState.FORCE_CONTROL_INSERT_STATE

        elif robot_control_state == RobotControlState.ROBOT_TASK_FINISH_STATE:   # 机械臂任务完成

            print("机械臂任务完成")
            return

test()







