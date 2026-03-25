'''
This module defines the basic data structure 
of the communication protocol. 
The communication protocol documentation see
该模块定义了基本数据结构
的通信协议。
通讯协议文档见
`Birdiebot Communication protocol <https://birdiebot.github.io/bubble_documentation/guide/%E6%9C%A8%E9%B8%A2%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.html>`__ .
'''

from collections import OrderedDict
import copy

# pose constant defination
# 姿势常数定义
HEAD_POSE = 0
D_ADDR_POSE = 1
ID_POSE = 2
LEN_POSE = 3
DATA_POSE = 4
SUM_CHECK_POSE = -2
ADD_CHECK_POSE = -1

BCP_TRUE = 1
BCP_FALSE = 0

IDX_BCP_TYPE = 0
IDX_VAL = 1
IDX_BCPID_RATIO = 2

# BCP Error code
# BCP 错误代码
CORE_MODULE_NORMAL = 0
CORE_MODULE_ERROR = 1
SYSTEM_WANGNING = 2
SYSTEM_INFO = 3

# BCP data pose and type defination
# BCP数据位姿和类型定义
IDX_BCPID = 0
IDX_BCP_DETAIL = 1

TYPE_FOR_CTYPE = {
    "char": "c",
    "int8": "b",
    "uint8": "B",
    "int16": "h",
    "uint16": "H",
    "int32": "i",
    "uint32": "I",
}

HEAD = 0xFF

D_ADDR = {
    "broadcast": 0x00,
    "mainfold": 0x01,
    "sentry_up": 0x02,
    "sentry_down": 0x03,
    "infantry": 0x04,
    "engineer": 0x05,
    "hero": 0x06,
    "air": 0x07,
    "radar": 0x08,
    "grather": 0x09,
    "standard": 0x10,
    "sentry" : 0x03, # 统一sentry_up与sentry_down发一个sentry（标志位0x02）
}

# referee systemrobot id defination
# 裁判系统机器人id定义
RefereeRobotID = {
    "red_hero": 1,
    "red_engineer": 2,
    "red_infantry_3": 3,
    "red_infantry_4": 4,
    "red_infantry_5": 5,
    "red_air": 6,
    "red_sentry": 7,
    "red_radar": 9,
    "red_outpost": 10,
    "red_base": 11,
    "blue_hero": 101,
    "blue_engineer": 102,
    "blue_infantry_3": 103,
    "blue_infantry_4": 104,
    "blue_infantry_5": 105,
    "blue_air": 106,
    "blue_sentry": 107,
    "blue_radar": 109,
    "blue_outpost": 110,
    "blue_base": 111,
}

# chassis data
# old api
chassis_info = OrderedDict()
chassis_info["chassis_vx"] = [TYPE_FOR_CTYPE["int8"], 0, 1]
chassis_info["chassis_vy"] = [TYPE_FOR_CTYPE["int8"], 0, 1]
chassis_info["chassis_angle"] = [TYPE_FOR_CTYPE["int32"], 0, 100]

# chassis control data
# In order to control chassis, define this frame
chassis_ctrl_info = OrderedDict()
chassis_ctrl_info["chassis_target_linear_x"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_ctrl_info["chassis_target_linear_y"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_ctrl_info["chassis_target_linear_z"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_ctrl_info["chassis_target_angular_x"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_ctrl_info["chassis_target_angular_y"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_ctrl_info["chassis_target_angular_z"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]

# chassis imu info
chassis_imu_info = OrderedDict()
chassis_imu_info["chassis_imu_eul_yaw"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_imu_info["chassis_imu_eul_pit"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_imu_info["chassis_imu_eul_rol"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_imu_info["chassis_imu_angle_x"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_imu_info["chassis_imu_angle_y"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_imu_info["chassis_imu_angle_z"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_imu_info["chassis_imu_acc_x"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_imu_info["chassis_imu_acc_y"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
chassis_imu_info["chassis_imu_acc_z"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]


# Current robot odometer information
odom_info = OrderedDict()
odom_info["odom_velocity_x"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
odom_info["odom_velocity_y"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
odom_info["odom_angular_velocity"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
odom_info["odom_position_x"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
odom_info["odom_position_y"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]
odom_info["odom_angle"] = [TYPE_FOR_CTYPE["int32"], 0, 10000]

# gimbal data
gimbal_info = OrderedDict()
# for gimbal control mode:
# 0. Absolute angle control
# 1. Relative angle control
gimbal_info["gimbal_ctrl_mode"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
gimbal_info["gimbal_yaw"] = [TYPE_FOR_CTYPE["int32"], 0, 1000]
gimbal_info["gimbal_pitch"] = [TYPE_FOR_CTYPE["int32"], 0, 1000]
gimbal_info["gimbal_roll"] = [TYPE_FOR_CTYPE["int32"], 0, 1000]
gimbal_info["gimbal_yaw_speed"] = [TYPE_FOR_CTYPE["int32"], 0, 1000]
gimbal_info["gimbal_pitch_speed"] = [TYPE_FOR_CTYPE["int32"], 0, 1000]
gimbal_info["gimbal_roll_speed"] = [TYPE_FOR_CTYPE["int32"], 0, 1000]

# core mode contrl
mode_info = OrderedDict()
mode_info["mode_ctrl"] = [TYPE_FOR_CTYPE["uint8"], BCP_FALSE, 1]

# robot shooter data
shooter_info = OrderedDict()
shooter_info["is_shoot"] = [TYPE_FOR_CTYPE["uint8"], BCP_FALSE, 1]
shooter_info["bullet_vel"] = [TYPE_FOR_CTYPE["int32"], 15, 1]
shooter_info["remain_bullet"] = [TYPE_FOR_CTYPE["int16"], 15, 1]

# onboard contrl
manifold_ctrl_info = OrderedDict()
manifold_ctrl_info["mode_ctrl"] = [TYPE_FOR_CTYPE["uint8"], 1, 1]

# referee system game data
game_status_info = OrderedDict()
game_status_info["game_type"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
game_status_info["game_progress"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
game_status_info["stage_remain_time"] = [TYPE_FOR_CTYPE["uint16"], 0., 100]

robot_HP_info = OrderedDict()
robot_HP_info["red_1_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["red_2_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["red_3_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["red_4_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["red_5_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["red_7_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["red_outpost_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["red_base_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["blue_1_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["blue_2_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["blue_3_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["blue_4_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["blue_5_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["blue_7_robot_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["blue_outpost_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
robot_HP_info["blue_base_HP"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]

ICRA_zone_info = OrderedDict()
ICRA_zone_info["F1_zone_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F1_zone_buff_debuff_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F2_zone_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F2_zone_buff_debuff_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F3_zone_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F3_zone_buff_debuff_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F4_zone_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F4_zone_buff_debuff_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F5_zone_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F5_zone_buff_debuff_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F6_zone_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["F6_zone_buff_debuff_status"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["red1_bullet_left"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["red2_bullet_left"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["blue1_bullet_left"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
ICRA_zone_info["blue2_bullet_left"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]

# game_mode(red or blue)
game_mode_info = OrderedDict()
game_mode_info["game_mode"] = [TYPE_FOR_CTYPE["uint8"], 1, 1]

# robot command
# refence system issued
robot_command_info = OrderedDict()
robot_command_info["target_position_x"] = [TYPE_FOR_CTYPE["int32"], 0, 1]
robot_command_info["target_position_y"] = [TYPE_FOR_CTYPE["int32"], 0, 1]
robot_command_info["target_position_z"] = [TYPE_FOR_CTYPE["int32"], 0, 1]
robot_command_info["commd_keyboard"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]
robot_command_info["target_robot_ID"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]

# radar command
# refence system issued
client_map_command_command_info = OrderedDict()
client_map_command_command_info["target_robot_ID"] = [TYPE_FOR_CTYPE["uint16"], 0, 1]
client_map_command_command_info["target_position_x"] = [TYPE_FOR_CTYPE["uint32"], 0, 1]
client_map_command_command_info["target_position_y"] = [TYPE_FOR_CTYPE["uint32"], 0, 1]

# heartbeat
heartbeat_info = OrderedDict()
heartbeat_info["heartbeat"] = [TYPE_FOR_CTYPE["uint8"], 0, 1]

ERROR_CODE = {
    "UPBOARD_OFFINE_ERROR": [CORE_MODULE_ERROR, 0x00],
    "IMAGE_OPEN_ERROR": [CORE_MODULE_ERROR, 0x01],
    "IMAGE_READ_ERROR": [CORE_MODULE_ERROR, 0x02],
    "TARGET_MISSING": [SYSTEM_WANGNING, 0x01],
    "WAIT_GAME_CHECK": [SYSTEM_INFO, 0x01],
    "WAIT_GAME_CHECK_RESET": [SYSTEM_INFO, 0x02]
}

# device error data
dev_error_info = OrderedDict()
dev_error_info["error_level"] = [TYPE_FOR_CTYPE["uint8"], 0, 1],
dev_error_info["error_module"] = [TYPE_FOR_CTYPE["uint8"], 0, 1],
dev_error_info["error_code"] = [TYPE_FOR_CTYPE["uint8"], 0, 1],

ID = {
    "chassis": [0x10, chassis_info],
    "chassis_odom": [0x11, odom_info],#1
    "chassis_ctrl": [0x12, chassis_ctrl_info],
    "chassis_imu": [0x13, chassis_imu_info],#1

    # === 哨兵姿态：上位机下发给 C 板的命令帧 (Command) ===
    "sentry_posture_cmd": [0x15, OrderedDict([
        ("posture", [TYPE_FOR_CTYPE["uint8"], 0, 1])  
    ])],

    # === 哨兵姿态：C 板上报给上位机的真实状态帧 (Feedback) ===
    "sentry_posture_fdb": [0x06, OrderedDict([
        ("posture", [TYPE_FOR_CTYPE["uint8"], 0, 1])  
    ])],

    "gimbal": [0x20, gimbal_info],

    # referee system game data, to onboard only
    "game_status": [0x30, game_status_info],
    "robot_HP": [0x31, robot_HP_info],
    "ICRA_buff_debuff_zone": [0x32, ICRA_zone_info],
    "game_mode": [0x33, game_mode_info],
    "robot_command": [0x34, robot_command_info],
    "client_map_command_command": [0x35, client_map_command_command_info],

    "barrel": [0x40, shooter_info],

    "manifold_ctrl": [0x50, manifold_ctrl_info],

    "mode": [0x60, mode_info],

    "dev_error": [0xE0, dev_error_info],
    "heartbeat": [0xF0, heartbeat_info],
}

##############birdiebot robot status defination##############

STATUS = {
    "manifold_ctrl": copy.deepcopy(manifold_ctrl_info),
    "gimbal": copy.deepcopy(gimbal_info),
    "chassis": copy.deepcopy(chassis_info),
    "chassis_odom": copy.deepcopy(odom_info),
    "chassis_imu": copy.deepcopy(chassis_imu_info),
    "chassis_ctrl": copy.deepcopy(chassis_ctrl_info),
    "barrel": copy.deepcopy(shooter_info),
    "game_status": copy.deepcopy(game_status_info),
    "game_mode": copy.deepcopy(game_mode_info),
    "robot_HP": copy.deepcopy(robot_HP_info),
    "ICRA_buff_debuff_zone": copy.deepcopy(ICRA_zone_info),
    "robot_command": copy.deepcopy(robot_command_info),
    "client_map_command_command": copy.deepcopy(client_map_command_command_info),
    # 拆分成两个容器，互不干涉
    "sentry_posture_cmd": copy.deepcopy(ID["sentry_posture_cmd"][1]),
    "sentry_posture_fdb": copy.deepcopy(ID["sentry_posture_fdb"][1]),
}

REALTIME_CALLBACK = {
    "gimbal": None,
    "barrel": None,
    "chassis_ctrl": None,
}


class BCP_FRAME():
    """BCP FRAME. 
    More infomation, see `Birdiebot Communication protocol <https://birdiebot.github.io/bubble_documentation/guide/%E6%9C%A8%E9%B8%A2%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.html>`__

    Attributes
    ----------
    info: `list`
        The data of frame.
    sumcheck: `int`
        Sum check of data frame.
    addcheck: `int`
        Additional of data frame.

    Methods
    -------
    setData(self, data: bytes)
        Add data to info list, and calculate check value at the same time.
        将数据添加到信息列表中，同时计算校验值。
    getData(self)
        Gets all values of the current data.
        获取当前数据的所有值。
    combineCheck(self)
        Set the final check values of the data frame.
        设置数据框的最终校验值。

    Examples
    --------
    >>> from bubble_protocol.protocol import BCP_FRAME
    >>> current_packet = BCP_FRAME()
    >>> current_packet.setData(55)
    >>> current_packet.combineCheck()
    >>> current_packet.getData()
    bytearray(b'777')
    >>> [i for i in current_packet.getData()]
    [55, 55, 55]
    """

    def __init__(self) -> None:
        self.info = bytearray([])
        self.sumcheck = 0
        self.addcheck = 0

    def setData(self, data: bytes) -> None:
        '''Add data to info list, and calculate check value at the same time.
            将数据添加到信息列表中，同时计算校验值。
        Parameters
        ------------
        data: `bytes`
            The data to be added.
        '''
        self.info.append(data)
        self.sumcheck += data
        self.addcheck += self.sumcheck

    def getData(self) -> bytearray:
        '''Gets all values of the current data.
            获取当前数据的所有值。
        Returns
        -----------
        `bytearray`
            all values of the current data.
        '''
        try:
            return self.info
        except Exception as e:
            print("get fream error:{}".format(e.args))
            return bytearray()

    def combineCheck(self) -> None:
        '''Set the final check values of the data frame.
            设置数据框的最终校验值。
        '''

        # print("sc: {}, ac: {}".format(int(self.sumcheck) & 0XFF, int(self.addcheck) & 0XFF))
        self.info.append(int(self.sumcheck) & 0XFF)
        self.info.append(int(self.addcheck) & 0XFF)


class BCP_TX_FRAME(BCP_FRAME):
    '''BCP transfer frame.
    if the data frame only needs to be received by local MCU, 
    you can use ``BCP_TX_FRAME``.
        BCP 传输帧。
        如果数据帧只需要被本地MCU接收，你可以使用``BCP_TX_FRAME``。
    See Also
    -----------
    :py:meth:`bubble_protocol.protocol.BCP_FRAME`.

    '''

    d_addr = None
    INFO = None

    def __init__(self) -> None:
        super().__init__()
        self.info = copy.deepcopy(self.INFO)
        self.sumcheck = HEAD + self.d_addr
        self.addcheck = HEAD + self.sumcheck
