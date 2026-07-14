#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
# import tf
import time
import json
import ast

import tf
import numpy as np  # 矩阵运行库
from dynamic_biped.msg import robotPhase
from dynamic_biped.msg import robotQVTau
from nlink_parser.msg import LinktrackNodeframe2
from kuavoRobotSDK import kuavo
from sensor_msgs.msg import JointState
from dynamic_biped.srv import moveToTarget, doAction
from dynamic_biped.msg import stopProcessing
from datetime import datetime
from action_process import ActionProcesser  # 载入胳膊点位

# ---常量设置---
# IMU调整阈值
imu_threshold = 5
# 机器人距离边缘距离
SIDE_THRESHOLD = 0.5
# 机器人距离目标点的偏移值
x_bias = 0.3
y_bias = 0.5
# 程序检查当前 uwb 位置频率
OPERATING_FREQUENCY = 1
# UWB filter size
UWB_FILTER_SIZE = 0.1
# 前进速度
FORWARD_SPEED = 0.5
# # 根据address_id读取address相关内容
# with open("/home/kuavo/harbin_ws/config/ADDRESS_LIST.json", "r") as load_f:
#     list = json.load(load_f)
#     ADDRESS_LIST = ast.literal_eval(list)

INITIAL_POS_X = 1.812999963760376
INITIAL_POS_Y = 3.0239999294281006

# 运动状态及对应数值
STAND_STATE = 'P_stand'
STAND_CODE = 0
WALK_STATE = 'P_walk'
WALK_CODE = 1
# 默认使用速度控制模式，对应值为 1
VEL_CONTROL_CODE = 1
DEFAULT_MASTER_ID = 0

# 为保证安全为区域一设置一个较大的偏移量
REGION_ONE_BIAS_X = 2.1
# 区域二默认起始 X 值为 16
REGION_TWO_BIAS_X = 16

ADDRESS_LIST = [
    # 起始点，东方红和功勋墙中间位置
    {
        "address_id": 0,
        "region": 0,
        "name": "QiShiDian",
        "coordinates": [1.812999963760376, 3.0239999294281006],
        "x_length": 3.312,
        "y_length": 20,
        "arrive_turn": 0,
        "depature_turn": 0,
        "transfor_coordinates": 0
    },
    # 第一个点，左边整流罩，右边火箭
    {
        "address_id": 1,
        "region": 0,
        "name": "ChangZhengHuoJian",
        "coordinates": [2.925189489773858, 13.966334738501086],
        "x_length": 5.157,
        "y_length": 20,
        "arrive_turn": 1,
        "depature_turn": -1,
        "transfor_coordinates": 0
    },
    # 第二个点，卫星区起点
    {
        "address_id": 2,
        "region": 0,
        "name": "RenZaoWeiXin",
        "coordinates": [3.321433177304822, 17.88599967956543],
        "x_length": 5.157,
        "y_length": 20,
        "arrive_turn": 1,
        "depature_turn": 0,
        "transfor_coordinates": 0
    },
    # 第三个点，卫星区讲解点
    {
        "address_id": 3,
        "region": 1,
        "name": "DaoHang",
        "coordinates": [18.285999298095703, 11.121000289916992],
        "x_length": 3.112,
        "y_length": 20,
        "arrive_turn": 2,
        "depature_turn": -2,
        "transfor_coordinates": 1,
        "transfor_matrix": [[0, 1], [-1, 0]],
        "x_coordinate_shift": -15.42,
        "y_coordinate_shift": 3.31
    },
]
class harbinCase:
    def __init__(self):
        self.robot_instance = kuavo("3_4_kuavo")
        self.action_processer = ActionProcesser()
        # 机器人状态初始化
        self.state = STAND_CODE
        self.yaw = 0
        # UWB参数初始化
        self.last_x, self.last_y = INITIAL_POS_X, INITIAL_POS_Y
        self.cur_x, self.cur_y = INITIAL_POS_X, INITIAL_POS_Y
        self.cur_region = 0
        self.cur_address = 0
        self.cur_counter = 0
        self.last_counter = 0

        self.uwb_data_x = INITIAL_POS_X
        self.uwb_data_y = INITIAL_POS_Y
        self.transfor_flag = False
        self.x_shift = 0
        self.y_shift = 0
        self.first_time_turn = 0
        # command srv返回值初始化，当前命令是否完成
        self.is_srv_finished = False
        # 机器人前进方向初始化
        self.cur_direction = 0  # 起始方向为 0，机器人向左转为正值
        self.tar_direction = 0

        # 服务端：接收指令，运动
        self.moving_srv = rospy.Service("/move_to_target", moveToTarget, self.srv_move_to_address)
        self.action_srv = rospy.Service("/doAction", doAction, self.srv_do_action)
        # topic：UWB数据处理
        self.uwb_sub = rospy.Subscriber("/nlink_linktrack_nodeframe2", LinktrackNodeframe2, self.uwb_callback)
        # topic:机器人位姿和运动状态
        self.robot_state_sub = rospy.Subscriber("/leju_robot_phase", robotPhase, self.state_callback)
        self.robot_QVTau_sub = rospy.Subscriber("/robot_q_v_tau", robotQVTau, self.imu_callback)
        # topic：监听是否暂停当前任务
        self.stopper_mark = rospy.Subscriber("/stopper_msg", stopProcessing, self.stopper_callback)

    # -----------------------订阅话题 callback函数-----------------------
    def srv_do_action(self, req):
        action_code = req.action_code
        self.stopper = False
        self.is_srv_finished = False

        action_list = self.action_processer.get_action_list()
        target_action = action_list[0]
        for index, action in enumerate(action_list):
            # 检查当前元素的 id 属性是否匹配
            if action['id'] == action_code:
                # 如果匹配，返回当前元素的下标
                target_action = action
                break
        
        print("exec action: {}".format(target_action["file_name"]))
        now = datetime.now()
        print("now: ", now)
        # print(target_action["hand_traj_data"])
        self.robot_instance.pub_kuavo_arm_with_time(target_action["time_data"], target_action["arm_traj_data"])
        cur_time = 0
        next_time = 0
        for index in range(len(target_action["time_data"])):
            next_time = target_action["time_data"][index]
            print("next time {}".format(next_time))
            self.robot_instance.srv_controlEndHand(target_action["hand_traj_data"][index])
            time.sleep(next_time - cur_time)
            cur_time = next_time

        self.is_srv_finished = True
        return self.is_srv_finished

    def srv_move_to_address(self, req):
        tar_address = req.address_code
        if tar_address >= len(ADDRESS_LIST):
            return False
        
        self.stopper = False
        self.is_srv_finished = False
        
        rospy.loginfo(f"接收到新的指令，从区域{ADDRESS_LIST[self.cur_address]['name']}前往区域{ADDRESS_LIST[tar_address]['name']}中")
        self.processing(tar_address)
        response = self.is_srv_finished
        print("cur_region: ", self.cur_region)
        print("return value: ", response)
        return response

    def state_callback(self, data):
        tmp_state = data.mainPhase
        if tmp_state != self.state:
            print("Action state changed, new state: {}".format(tmp_state))
            self.state = tmp_state

    def imu_callback(self, data):
        tmp_roll, tmp_pitch, tmp_yaw = tf.transformations.euler_from_quaternion([data.q[1], data.q[2], data.q[3], data.q[0]])
        self.yaw = tmp_yaw

    def stopper_callback(self, data):
        self.stopper = bool(data)
        return True

    def uwb_callback(self, data):
        # UWB数据转化坐标系
        # 全局坐标系下在第一区域沿 y 轴前进，第二区域沿 x 轴前进
        # 为保证处理一致，将第二区域的 x、y 对调
        # 另外不会在每次收到数据的时候即可进行过滤，将过滤放在每次计算下一秒执行策略时
        tmp_x, tmp_y = data.pos_3d[0], data.pos_3d[1]
        if (not self.transfor_flag):
            self.uwb_data_x = tmp_x
            self.uwb_data_y = tmp_y
        else:
            self.uwb_data_x = tmp_y
            self.uwb_data_y = tmp_x
        
        # print("raw data: x {}, y {}".format(tmp_x, tmp_y))

    def update_robot_pos(self):
        tmp_x = self.uwb_data_x
        tmp_y = self.uwb_data_y
        print("raw uwb data, x: {}, y: {}".format(tmp_x, tmp_y))

        # 第一次读取数据时直接更新
        if self.cur_x == INITIAL_POS_X and self.cur_y == INITIAL_POS_Y:
            self.cur_x = tmp_x
            self.cur_y = tmp_y

        # 在执行策略中，由于 y 轴读数较准，仅执行过滤抖动
        if abs(self.cur_y - tmp_y) > UWB_FILTER_SIZE:
            self.last_y = self.cur_y
            self.cur_y = tmp_y
        
        if abs(self.cur_x - tmp_x) > UWB_FILTER_SIZE:
            if abs(self.cur_x - tmp_x) < 0.5:
                self.last_x = self.cur_x
                self.cur_x = tmp_x
            else:
                # 数据出现较大波动，需要额外处理
                # 根据最新数值进行补偿，但不直接进行赋值
                self.last_x = self.cur_x
                self.cur_x = self.cur_x + (0.1 if self.cur_x < tmp_x else -0.1)
        
        print("cur robot x {}, cur robot y {}".format(self.cur_x, self.cur_y))
        

   # -----------------------机器人状态和运动处理-----------------------
    def change_robot_state(self, new_state):
        print("Change state to {}".format(new_state))
        self.robot_instance.set_robot_Phases(DEFAULT_MASTER_ID, new_state, '')

    def set_robot_vel(self, vel_y=0.2, vel_x=0.1, yaw=0.0):
        # 在 uwb 坐标系中 y 轴为前后方向
        # set_walk_speed 第二个参数为前后速度，正值向前运动，负值向后运动
        # 在 uwb 坐标系中 x 轴为左右方向
        # set_walk_speed 第三个参数为左右速度，正值向左运动，负值向右运动
        # 在 IMU 坐标系中 左边为正，右边为负
        # set_walk_speed 第四个参数为imu显示偏移角度，正值向左，负值向右
        self.robot_instance.set_walk_speed(VEL_CONTROL_CODE, vel_y, vel_x, yaw)

    def turn_robot(self, angle):
        # set_walk_speed 第四个参数为转动角度，正值向左转动，负值向右转动
        if self.state == STAND_CODE:
            # 对于下位机保护程序的异常处理
            self.change_robot_state(WALK_STATE)
            time.sleep(1.75)
        print("转向角度：", angle)
        self.robot_instance.set_walk_speed(VEL_CONTROL_CODE, 0.0, 0.0, angle)

    def arm_control(self, arm_traj):
        self.robot_instance.set_arm_traj_position(arm_traj)

    def stop_robot(self):
        self.set_robot_vel(0.0, 0.0, 0.0)
        time.sleep(2)
        self.change_robot_state(STAND_STATE)
        time.sleep(2)

    def coordinate_transform(self, x, y):
        pre_coordinates = np.array([x, y])
        post_coordinates = np.array([[0, 1], [-1, 0]])@pre_coordinates
        tmp_x, tmp_y = post_coordinates
        post_x = tmp_x - 15.42
        post_y = tmp_y + 3.31
        return post_x, post_y
    
    # -----------------------机器人转向-----------------------
    def is_at_target(self, tar_x, tar_y, tar_address):
        print("cur_x: {}, cur_y: {}".format(self.cur_x, self.cur_y))
        print("tar_x: {}, tar_y: {}".format(tar_x, tar_y))   
        distance_x = abs(self.cur_x - tar_x)
        distance_y = abs(self.cur_y - tar_y)
        if distance_x < x_bias and (distance_y < y_bias or self.cur_y > tar_y):
            return True
        elif self.cur_y > tar_y and tar_address == 3:
            return True
        else:
            return False

    def move_to_target(self, target_x, target_y, x_bound, y_bound, tar_address):
        self.update_robot_pos()
        print("print in processing: cur_x: {}, cur_y: {}".format(self.cur_x, self.cur_y))
        # 设置机器人进入行走模式，原地踏步 2 秒后开始运动
        self.change_robot_state(WALK_STATE)
        time.sleep(1.75)
        rate = rospy.Rate(OPERATING_FREQUENCY)
        # ---始终沿y轴运动---
        while (not rospy.is_shutdown()) and not (self.is_at_target(target_x, target_y, tar_address)):
            if (not self.stopper):
                vel_x = 0.0
                vel_y = 0.0
                # 异常情况:
                # 机器人保护后会自动进入站立状态，使其继续行走
                if self.state == STAND_CODE:
                    self.change_robot_state(WALK_STATE)
                    time.sleep(2)
                # 正常状态：
                else:
                    self.update_robot_pos()
                    # 优先判断机器人是否会碰到两边
                    if tar_address < 3:
                        if (self.cur_x < SIDE_THRESHOLD + REGION_ONE_BIAS_X):
                            vel_x = 0.1
                            print("左移")
                        elif (self.cur_x > x_bound - SIDE_THRESHOLD):
                            print("print x bound: {}".format(x_bound))
                            vel_x = -0.1
                            print("右移")
                        # 不会碰到的话向前移动
                        else:
                            # 偏离中心位置
                            if target_y - self.cur_y > y_bias:
                                vel_y = FORWARD_SPEED
                                print("向前")
                            # elif self.cur_y - target_y > y_bias:
                            #     vel_y = -0.1
                            #     print("向后")
                            else:
                                if (self.cur_x - target_x > x_bias):
                                    vel_x = -0.1
                                    print("向右调整")
                                elif (target_x - self.cur_x > x_bias):
                                    vel_x = 0.1
                                    print("向左调整")
                    else:
                        # 偏离中心位置，过道较窄需要额外处理
                        if self.cur_x < REGION_TWO_BIAS_X + SIDE_THRESHOLD + 1.5:
                            vel_x = -0.1
                            print("向右移动")
                        elif self.cur_x > REGION_TWO_BIAS_X + x_bound - SIDE_THRESHOLD + 0.5:
                            vel_x = 0.1
                            print("向左移动")
                        else:
                            if target_y - self.cur_y > y_bias:
                                vel_y = FORWARD_SPEED
                                print("向前")
                            # elif self.cur_y - target_y > y_bias:
                            #     vel_y = -0.1
                            #     print("向后")
                self.set_robot_vel(vel_y, vel_x, 0.0)
                rate.sleep()
                # print("x: ", self.cur_x)
                # print("y: ", self.cur_y)
                print("偏移：", self.yaw)
                print("direction: ", self.cur_direction)
                print("坐标系是否转换： ", self.transfor_flag)
                print("cur_region: ", self.cur_region)
            else:
                self.stop_robot()
                print("程序中断，机器人待命")
                print("x: ", self.cur_x)
                print("y: ", self.cur_y)
                print("偏移：", self.yaw)
                print("direction: ", self.cur_direction)
                print("坐标系是否转换： ", self.transfor_flag)
                print("cur_region: ", self.cur_region)
                self.cur_counter = 0
                return
            # 抵达指定位置，原地踏步 2 秒然后恢复站立
        self.stop_robot()
        print("到达moving目标点")
    
    
    def pre_moving(self):
        # 从起始点开始运动，向移动到中间位置
        self.change_robot_state(WALK_STATE)
        time.sleep(2.1)
        self.set_robot_vel(FORWARD_SPEED, 0.0, 0.0)
        time.sleep(4)
        self.set_robot_vel(0.0, 0.0, 0.0)

        time.sleep(1)
        for i in range(10):
            time.sleep(0.5)
            self.turn_robot(-9)

    def processing(self, tar_address):
        #只有在讲解点结束服务，重启服务后，才会转回之前方向，在0号点（counter = 0)，和服务被stopper终端时（counter = 0), 会直接跳过

        if self.cur_counter != 0:
            turn_direction = ADDRESS_LIST[tar_address-1]["depature_turn"]
            if turn_direction != 0:
                self.change_robot_state(WALK_STATE)
                time.sleep(1.5)
                for i in range(10*abs(turn_direction)):
                    time.sleep(0.5)
                    self.turn_robot(9*(turn_direction)/abs(turn_direction))
                    print("机器人启动，调整方向中")
                self.cur_direction = self.cur_direction + turn_direction

        # 第一次移动（前往1）先沿x轴走到中线位置
        if tar_address == 1:
            self.pre_moving()

        # 获取坐标点信息
        self.cur_region = ADDRESS_LIST[tar_address]["region"]
        tar_x = ADDRESS_LIST[tar_address]["coordinates"][0]
        tar_y = ADDRESS_LIST[tar_address]["coordinates"][1]
        x_bound = ADDRESS_LIST[tar_address]["x_length"]
        y_bound = ADDRESS_LIST[tar_address]["y_length"]

        back_up_y = self.cur_y
        back_up_x = self.cur_x
        self.transfor_flag = ADDRESS_LIST[tar_address]["transfor_coordinates"]
        if tar_address == 3:
            # 处理切换坐标轴事宜，提前为 x 轴赋值
            print("ready for change axis....")
            time.sleep(1)
            self.cur_x = back_up_y
            self.cur_y = back_up_x
            time.sleep(1)

        self.move_to_target(tar_x, tar_y, x_bound, y_bound, tar_address)

        # 正常结束
        if (not self.stopper):
            turn_direction = ADDRESS_LIST[tar_address]["arrive_turn"]
            self.change_robot_state(WALK_STATE)
            time.sleep(1.5)
            for i in range(11*abs(turn_direction)):
                time.sleep(0.5)
                self.turn_robot(9*((turn_direction)/abs(turn_direction)))
                print("到达讲解点转身中")
            self.cur_direction = self.cur_direction + turn_direction
            self.cur_address = tar_address
            self.stop_robot()
            self.is_srv_finished = True
            print("服务结束")
            self.cur_counter += 1
            print(self.cur_counter)
        # stop跳出结束
        else:
            self.stop_robot()
            self.is_srv_finished = False
            self.cur_counter = 0       
        time.sleep(2)


if __name__ == '__main__':
    rospy.init_node('main', anonymous=True)
    handler = harbinCase()

    while (not rospy.is_shutdown()):
        # handler.update_robot_pos()
        rospy.Rate(1)
